
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <limits.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "indicadores.pb.h"
#include "lsm9ds1_reg.h"
#include "math.h"
#include "normalizacion.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                10000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

//#define SAMPLES_IN_BUFFER   2
//static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];

#define PERIODO_MUESTREO  APP_TIMER_TICKS(400)   //muestreo
APP_TIMER_DEF(m_timer_muestreo_id);

#define PERIODO_TIMER_BLINK  APP_TIMER_TICKS(500)   //señal lumínica
APP_TIMER_DEF(m_timer_blink_id);

#define PERIODO_CONECTADO_SIN_USO  APP_TIMER_TICKS(100000)   //tiempo de espera para la desconexión y apagado en caso de no detectar presión en la punta (uso)
APP_TIMER_DEF(m_timer_inactividad_id);

bool captura = false;

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

stmdev_ctx_t dev_ctx_imu;
stmdev_ctx_t dev_ctx_mag;

float roll, pitch;

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#define SPI_INSTANCE  1                                                 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);    /**< SPI instance. */
static volatile bool spi_xfer_done;                                     /**< Flag used to indicate that SPI instance completed the transfer. */

#define ADC_CONFIG {0xBF, 0x1A}
#define ADC_DUMMY {0X08, 0X08}
#define RX_SIZE 12
static uint8_t       m_tx_buf[] = ADC_CONFIG;           /**< TX buffer. */
static uint8_t       m_tx_dummy_buf[] = ADC_DUMMY;
static uint8_t       m_rx_buf[RX_SIZE];    /**< RX buffer. */

static uint8_t       results[8];
static uint16_t      canales = 6; 


/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
char tx_buffer[100];


/* TWI instance. */
 static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
uint8_t register_address = LSM9DS1_WHO_AM_I;    //Address of the who am i register to be read

//static lsm9ds1_id_t whoamI;

void imu_off(void);
void habilitar_muestreo(void);
void deshabilitar_muestreo(void);
static void app_timer_muestreo_stop(void);


Trama tramas_actividad[10] = {Trama_init_zero};
uint8_t cnt_trama = 0;


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint16_t reg16 = reg;   // ???
  err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, (uint8_t *)&reg16, 1, true);
  if (NRF_SUCCESS != err_code){
    return 0;
  }
  err_code = nrf_drv_twi_rx(&m_twi, *i2c_address, bufp, len); 
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
  uint8_t *i2c_address = handle;
  ret_code_t err_code;
  uint8_t buffer[1 + len];      // se crea un nuevo buffer para incluir la direccion del registro al inicio
  memcpy(buffer, &reg, 1);
  memcpy(buffer + 1, bufp, len);
    err_code = nrf_drv_twi_tx(&m_twi, *i2c_address, buffer, len + 1, true);
//    if(err_code == NRF_SUCCESS){
//      NRF_LOG_INFO("I2C WRITTEN");
//    }
//    NRF_LOG_FLUSH();
  return 0;
}


/*********************************************/
void transmision(void){
    uint8_t buffer[128];
    uint16_t message_length;
    bool status;
    uint32_t err_code;
    
     NRF_LOG_INFO("presiones");

    for (int i = 0; i < cnt_trama; i++){
        NRF_LOG_INFO("%d\n",tramas_actividad[i].presion);
    }
    Trama trama = evaluar_tramas_actividad(tramas_actividad, cnt_trama);

    NRF_LOG_INFO("presion: %d, agarre: %d, dir: %d\n",trama.presion, trama.agarre, trama.direccionalidad);

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    //trama.has_lapiz = true; 
    #if defined(LAPIZ_TRI)
        trama.lapiz = Trama_Lapiz_TRI;
    #elif defined(LAPIZ_HEX)
        trama.lapiz = Trama_Lapiz_HEX;
    #else
        #error "Se necesita definir el modelo de lapiz"
    #endif

//    trama.has_bat_lapiz = true;
//    //trama.bat_lapiz = 87;
//    trama.has_agarre = true;
//    //trama.agarre = Trama_Agarre_DIGITAL;
//    trama.has_presion = true;
//    //trama.presion = Trama_Presion_HIPERTONICO;
//    trama.has_animo = true;
//    trama.animo = Trama_Animo_CALMADO;
//    trama.has_direccionalidad = true;
//    //trama.direccionalidad = Trama_Direccionalidad_BUENA;
//    trama.has_movimiento = true;
//    //trama.movimiento = Trama_Movimiento_MANO;

    
        
    status = pb_encode(&stream, Trama_fields, &trama);

    message_length = stream.bytes_written;
        
    if (!status)
    {
        NRF_LOG_INFO("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    } else{
        err_code = ble_nus_data_send(&m_nus, buffer, &message_length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
        NRF_LOG_INFO("ENVIADO TRAMA");
    }

    cnt_trama = 0;
}



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint8_t num_act;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS: %d bytes",  p_evt->params.rx_data.length);
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        num_act = *p_evt->params.rx_data.p_data & 0xF;
        if(num_act < 10){
        transmision();
        }

    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    imu_off();
    nrf_gpio_pin_clear(AD7298_PD_RST);
    nrf_gpio_pin_clear(BAT_MON_EN);

    app_timer_muestreo_stop();
//    nrfx_saadc_uninit();
//    nrf_drv_spi_uninit(&spi);
//    nrf_drv_twi_uninit(&m_twi);
//
//    nrf_gpio_cfg_default(LED_1);
//    nrf_gpio_cfg_default(LED_2);
//    nrf_gpio_cfg_default(LED_3);
//    nrf_gpio_cfg_default(BUZZER);
//     nrf_gpio_cfg_default(BAT_CHRG_STAT);
//
//    nrf_gpio_cfg_default(SCL_SPC);
//    nrf_gpio_cfg_default(SDO_AG);
//    nrf_gpio_cfg_default(SDA_SDI_SDO);
//    nrf_gpio_cfg_default(SDO_M);
//    nrf_gpio_cfg_default(CS_M);
//    nrf_gpio_cfg_default(CS_AG);
// 
//    nrf_gpio_cfg_default(SPIM0_SCK_PIN);
//    nrf_gpio_cfg_default(SPIM0_MOSI_PIN);
//    nrf_gpio_cfg_default(SPIM0_MISO_PIN);
//    nrf_gpio_cfg_default(SPIM0_SS_PIN);
//
//    nrf_gpio_cfg_default(VBAT);
//    nrf_gpio_cfg_default(FSS_N);
//    nrf_gpio_cfg_default(FSS_P);


    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);

      NRF_POWER -> SYSTEMOFF = 1; 
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("ADV expirado, a dormir");
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            //estado =  CONECTADO;

            //iniciar toma de datos (temporizador de muestreo)
            habilitar_muestreo();
            
            

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            //estado = DESCONECTADO;
            deshabilitar_muestreo();


            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
     if (pin == VBUS_DETECT && action == NRF_GPIOTE_POLARITY_LOTOHI){
        //nrf_gpio_pin_toggle(LED_3);
        NRF_LOG_INFO("SENSADO");
        transmision();
     }
     else
        NRF_LOG_INFO("OTRO EVENTO");
}

static void timer_muestreo_timeout_handler(void * p_context)
{
    uint32_t  err_code;
    nrf_saadc_value_t fss;
    nrf_saadc_value_t bat;

    nrfx_saadc_sample_convert(0, &bat);
    NRF_LOG_INFO("\nbat: %d\n", bat);

    nrfx_saadc_sample_convert(1, &fss);
    NRF_LOG_INFO("\nfss: %d\n", fss);

    

    NRF_LOG_FLUSH();

    if(captura){
        NRF_LOG_INFO("CAPTURANDO");
          
          Trama trama_captura = Trama_init_zero;

          trama_captura.presion = presion(fss);
          //trama_captura.bateria = bateria(bat);
            
          /* Read device status register */
           lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

          if ( reg.status_imu.xlda && reg.status_imu.gda ){
            /* Read imu data */
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

            lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
            lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

            acceleration_mg[0] = lsm9ds1_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] = lsm9ds1_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] = lsm9ds1_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

            angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

            roll = atan2f(acceleration_mg[1], acceleration_mg[2]) * 180 / M_PI;
            pitch = atanf( (-1 * acceleration_mg[0]) / sqrtf(powf(acceleration_mg[1], 2) + powf(acceleration_mg[2], 2 ))) * 180 / M_PI;

            NRF_LOG_INFO("\nroll = %d\npitch = %d\ndir = %d\n", (int)roll, (int)pitch, direccionalidad(pitch, roll) );

            trama_captura.direccionalidad = direccionalidad(pitch, roll);

           }

            memset(m_rx_buf, 0, RX_SIZE);  // Reset rx buffer and transfer done flag

            spi_xfer_done = false;

            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, NULL, 0));  // adc_config

            while (!spi_xfer_done)
            {
                __WFE();
            }

            spi_xfer_done = false;

            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_dummy_buf, 2, NULL, 0));  // dummy transmit, wait conversion

             while (!spi_xfer_done)
            {
                __WFE();
            }


            for (int i = 0; i < RX_SIZE; i++) {
    
              spi_xfer_done = false;

              APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf + 2 * i, 2));    // rx results

               while (!spi_xfer_done)
              {
                  __WFE();
              }


            }
           NRF_LOG_HEXDUMP_INFO(m_rx_buf, 12);
            NRF_LOG_INFO("\nagarre: %d\n ",agarre(m_rx_buf));
          
           trama_captura.agarre = agarre(m_rx_buf);
            
           if(cnt_trama < 10){ 
              tramas_actividad[cnt_trama] = trama_captura;
              cnt_trama++;
            }

    }
    else 
        NRF_LOG_INFO("NO-CAP");

}

static void timer_blink_timeout_handler(void * p_context)
{

    nrf_gpio_pin_clear(LED_2);

}

static void timer_inactividad_timeout_handler(void * p_context)
{   
    NRF_LOG_INFO("Inactivo por demasiado tiempo");
    nrf_gpio_pin_set(LED_1);
    nrf_delay_ms(1000);
    nrf_gpio_pin_clear(LED_1);

    sleep_mode_enter();
}

static void app_timers_init(void)
{
    ret_code_t err_code;
  
    err_code = app_timer_create(&m_timer_muestreo_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_muestreo_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer_inactividad_id,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            timer_inactividad_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer_blink_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_blink_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void app_timer_muestreo_start(void){
    ret_code_t err_code;

    err_code = app_timer_start(m_timer_muestreo_id, PERIODO_MUESTREO, NULL);
    APP_ERROR_CHECK(err_code);
}

static void app_timer_muestreo_stop(void){
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_muestreo_id);
    APP_ERROR_CHECK(err_code);
}

static void app_timer_inactividad_start(void){
    ret_code_t err_code;
    app_timer_start(m_timer_inactividad_id, PERIODO_CONECTADO_SIN_USO, NULL);
    APP_ERROR_CHECK(err_code);
}

static void app_timer_inactividad_stop(void){
    ret_code_t err_code;
    app_timer_stop(m_timer_inactividad_id);
    APP_ERROR_CHECK(err_code);
}

static void blink(void)
{
    ret_code_t err_code;
    nrf_gpio_pin_set(LED_2);
    err_code = app_timer_start(m_timer_blink_id, PERIODO_TIMER_BLINK, NULL);
    APP_ERROR_CHECK(err_code);
}

//static void bat_check(bool blink){
//    uint32_t  err_code;
//    nrf_saadc_value_t fss;
//
//    err_code = nrfx_saadc_sample_convert(1, &fss);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("\nfss: %d\n", fss); 
//    if (blink){
//        if (fss < )
//    
//    }
//}

void input_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
    if(pin == INT2_AG){
        nrf_gpio_pin_toggle(LED_1);
        //NRF_LOG_INFO("DRDY_XL");
    }
}
void gpio_config(void){

    nrf_gpio_cfg_output(LED_3);   //led pruebas
    nrf_gpio_pin_clear(LED_3);

    nrf_gpio_cfg_output(BAT_MON_EN);
    nrf_gpio_pin_set(BAT_MON_EN);

    nrf_gpio_cfg_output(AD7298_PD_RST);
    nrf_gpio_pin_set(AD7298_PD_RST);

    nrfx_gpiote_in_config_t vbus_pin_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    nrfx_err_t err_code = nrfx_gpiote_in_init(VBUS_DETECT, &vbus_pin_config, gpio_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(VBUS_DETECT, true);

//    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
//    //config.pull = NRF_GPIO_PIN_PULLDOWN;
//    err_code = nrfx_gpiote_in_init(INT2_AG, &config, input_pin_handler);
//    APP_ERROR_CHECK(err_code);
//
//    nrfx_gpiote_in_event_enable(INT2_AG, true);

//    nrf_gpio_cfg_output(CS_AG);
//    nrf_gpio_cfg_output(CS_M);
//    nrf_gpio_cfg_output(SDO_AG);
//    nrf_gpio_cfg_output(SDO_M);

    nrf_gpio_cfg( CS_AG, 
                  NRF_GPIO_PIN_DIR_OUTPUT, 
                  NRF_GPIO_PIN_INPUT_DISCONNECT, 
                  NRF_GPIO_PIN_PULLUP,
                  NRF_GPIO_PIN_S0S1, 
                  NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg( CS_M, 
                  NRF_GPIO_PIN_DIR_OUTPUT, 
                  NRF_GPIO_PIN_INPUT_DISCONNECT, 
                  NRF_GPIO_PIN_PULLUP,
                  NRF_GPIO_PIN_S0S1, 
                  NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg( SDO_AG, 
                  NRF_GPIO_PIN_DIR_OUTPUT, 
                  NRF_GPIO_PIN_INPUT_DISCONNECT, 
                  NRF_GPIO_PIN_PULLUP,
                  NRF_GPIO_PIN_S0S1, 
                  NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg( SDO_M, 
                  NRF_GPIO_PIN_DIR_OUTPUT, 
                  NRF_GPIO_PIN_INPUT_DISCONNECT, 
                  NRF_GPIO_PIN_PULLUP,
                  NRF_GPIO_PIN_S0S1, 
                  NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_pin_set(CS_AG);
    nrf_gpio_pin_set(CS_M);
    nrf_gpio_pin_set(SDO_AG);
    nrf_gpio_pin_set(SDO_M);
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_SPC,
       .sda                = SDA_SDI_SDO,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void imu_off(){
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_OFF);
    //lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_POWER_DOWN);
}

void init_imu(){
  

 }   

/*********** ADC ***********/

void saadc_callback(nrfx_saadc_evt_t const * p_event)    //llamada cuando el buffer se llena (DONE) o se alcanza un límite (LIMIT)
{
    static int m_adc_evt_counter = 0;
    if (p_event->type == NRFX_SAADC_EVT_DONE)  //BUFFER LLENO
    {
        ret_code_t err_code;
            nrf_gpio_pin_toggle(LED_3);
//        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);    //alternar de buffer
//        APP_ERROR_CHECK(err_code);

//        int i;
//        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);
//
//        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
//        {
//            NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
//        }
//        trama.bat_lapiz = (p_event->data.done.p_buffer[1] - 200) * 2 ;
//        trama.presion = presion((int16_t)p_event->data.done.p_buffer[0]);

        m_adc_evt_counter++;
    } 
    else if (p_event->type == NRFX_SAADC_EVT_LIMIT)
    {
        if (p_event->data.limit.channel == 0){
          ret_code_t err_code;

          NRF_LOG_INFO("Bateria baja, a dormir");
          nrf_gpio_pin_set(LED_1);
          nrf_delay_ms(1000);
          nrf_gpio_pin_clear(LED_1);
          
          
          sleep_mode_enter();
        }
        else if (p_event->data.limit.channel == 1){

          if (p_event->data.limit.limit_type == NRF_SAADC_LIMIT_LOW){ 
            app_timer_inactividad_start();
            NRF_LOG_INFO("Lapiz en espera");
            nrfx_saadc_limits_set(1, NRFX_SAADC_LIMITL_DISABLED, 0);
            captura = false;
            
          } 
          else{
            app_timer_inactividad_stop();
            NRF_LOG_INFO("Lapiz en uso");
            nrfx_saadc_limits_set(1, 0, NRFX_SAADC_LIMITH_DISABLED);
            captura = true;
          }
        }
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t bat_channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VBAT);

    bat_channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;    
    bat_channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;      
    bat_channel_config.gain       = NRF_SAADC_GAIN2;                
    bat_channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;     
    bat_channel_config.acq_time   = NRF_SAADC_ACQTIME_40US;           
    bat_channel_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;     
    bat_channel_config.burst      = NRF_SAADC_BURST_DISABLED;        
    //bat_channel_config.pin_p      = (nrf_saadc_input_t)(PIN_P),       
    bat_channel_config.pin_n      = NRF_SAADC_INPUT_DISABLED;

    nrf_saadc_channel_config_t fss_channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(FSS_P, FSS_N);

    fss_channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                       
    fss_channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                       
    fss_channel_config.gain       = NRF_SAADC_GAIN4;                                 
    fss_channel_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;                      
    fss_channel_config.acq_time   = NRF_SAADC_ACQTIME_20US;                            
    fss_channel_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;                       
    fss_channel_config.burst      = NRF_SAADC_BURST_DISABLED;                          
    //fss_channel_config.pin_p      = (nrf_saadc_input_t)(PIN_P),                        
    //fss_channel_config.pin_n      = (nrf_saadc_input_t)(PIN_N) 
    
    nrfx_saadc_config_t saadc_config = {                                                                         
    .resolution         = (nrf_saadc_resolution_t)NRF_SAADC_RESOLUTION_8BIT, 
    .oversample         = (nrf_saadc_oversample_t)NRF_SAADC_OVERSAMPLE_DISABLED, 
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,                       
    .low_power_mode     = NRFX_SAADC_CONFIG_LP_MODE                             
    };

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &bat_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(1, &fss_channel_config);
    APP_ERROR_CHECK(err_code);

    nrfx_saadc_limits_set(0, 180, NRFX_SAADC_LIMITH_DISABLED);
    //nrfx_saadc_limits_set(1, NRFX_SAADC_LIMITL_DISABLED, 0);

//    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);       //se utilizan dos buffers para no perder muestras en el procesamiento cuando un buffer está lleno
//    APP_ERROR_CHECK(err_code);


}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    //static
    spi_xfer_done = true;



}

void spi_init(void){
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.mode = NRF_DRV_SPI_MODE_2;
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    spi_config.ss_pin   = SPIM0_SS_PIN;
    spi_config.miso_pin = SPIM0_MISO_PIN;
    spi_config.mosi_pin = SPIM0_MOSI_PIN;
    spi_config.sck_pin  = SPIM0_SCK_PIN;
    spi_config.irq_priority = 3;    //elevar la prioridad de interrupciones SPI porque las transferencias son llamadas desde el event_handler del timer
    spi_config.orc = 0x7E;

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
 }


void habilitar_muestreo(){
    nrfx_saadc_limits_set(1, NRFX_SAADC_LIMITL_DISABLED, 0);    // Activar limite superior para la presión (detectar uso del lápiz)
    app_timer_muestreo_start();
}

void deshabilitar_muestreo(){
    captura = false;
    nrfx_saadc_limits_set(1, NRFX_SAADC_LIMITL_DISABLED, NRFX_SAADC_LIMITH_DISABLED);    // Desactivar límites presión (detectar uso del lápiz)
    
}





/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    uint8_t sample_data;
    bool detected_device = false;

    // Initialize.
    //uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    gpio_config();
    app_timers_init();
    //app_timers_start();
    
    spi_init();
    saadc_init();
    twi_init();
    //init_imu();

/* Initialize inertial sensors (IMU) driver interface */
    uint8_t i2c_add_mag = LSM9DS1_MAG_I2C_ADD_H >> 1;
    //stmdev_ctx_t dev_ctx_mag;
    dev_ctx_mag.write_reg = platform_write;
    dev_ctx_mag.read_reg = platform_read;
    dev_ctx_mag.handle = (void*)&i2c_add_mag;

    /* Initialize magnetic sensors driver interface */
    uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H >> 1;
    //stmdev_ctx_t dev_ctx_imu;
    dev_ctx_imu.write_reg = platform_write;
    dev_ctx_imu.read_reg = platform_read;
    dev_ctx_imu.handle = (void*)&i2c_add_imu;

    /* Check device ID */
    lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
    if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
      while(1){
        /* manage here device not found */
        //NRF_LOG_INFO("\r\nCannot find the LSM9DS1.********\r\n");
        //NRF_LOG_FLUSH();
        NRF_LOG_INFO("\r\nCannot find the LSM9DS1.********\r\n");
      }
    }
    NRF_LOG_INFO("Who am I register [IMU]: 0x%x [MAG]: 0x%x \r\n", whoamI.imu, whoamI.mag);   

    /* Restore default configuration */
    lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
    do {
      lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

    /* Set full scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

    /* Configure filtering chain - See datasheet for filtering chain details */
    /* Accelerometer filtering chain */
    lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
    lsm9ds1_xl_filter_int_path_set(&dev_ctx_imu, LSM9DS1_HP_DIS);
    /* Gyroscope filtering chain */
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_14Hz9);
    //lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    
    //lsm9ds1_pin_logic_set(&dev_ctx_imu, LSM9DS1_LOGIC_AND);
    lsm9ds1_pin_notification_set(&dev_ctx_mag, &dev_ctx_imu, LSM9DS1_INT_PULSED);
    lsm9ds1_pin_mode_set(&dev_ctx_imu, LSM9DS1_PUSH_PULL);
    lsm9ds1_pin_polarity_set(&dev_ctx_mag, &dev_ctx_imu, LSM9DS1_ACTIVE_HIGH);

//    lsm9ds1_xl_trshld_en_t thr_en;
//    thr_en.xlie_xl             = 1;
//    thr_en.xhie_xl             = 1;
//    thr_en.ylie_xl             = 1;
//    thr_en.yhie_xl             = 1;
//    thr_en.zlie_xl             = 1;
//    thr_en.zhie_xl             = 1;
//    lsm9ds1_xl_trshld_axis_set(&dev_ctx_imu, thr_en);
//    uint8_t xl_thr[] = {0x2C, 0x2C, 0x2C};
//    lsm9ds1_6d_threshold_set(&dev_ctx_imu, xl_thr);
//    lsm9ds1_6d_mode_set(&dev_ctx_imu, LSM9DS1_6D_MOVE_RECO);
//
//    lsm9ds1_pin_int1_route_t int1_config;
//    int1_config.int1_ig_xl = 1;

//    lsm9ds1_pin_int2_route_t int2_config;
//    int2_config.int2_drdy_xl = 1;

    lsm9ds1_xl_trshld_min_sample_set(&dev_ctx_imu, 0x1F);

    //lsm9ds1_pin_int1_route_set(&dev_ctx_imu, int1_config);
//    lsm9ds1_pin_int2_route_set(&dev_ctx_imu, int2_config);



    NRF_LOG_INFO("LAPIZ GIIATA INICIADO");
    advertising_start();


    for (;;)
    {



           
           

            
            NRF_LOG_FLUSH();
            idle_state_handle();
        
    }
}


