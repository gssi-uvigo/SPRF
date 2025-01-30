
#include "normalizacion.h"
#include <stdlib.h>


/* Resultado del ADC por canal
   [ADD3, ADD2, ADD1, ADD0, DB9, DB8, DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0, DC, DC]
   [        Se toma solo este byte       ]
*/

static int ad7298_parse(uint8_t *buff, uint8_t ch_num, uint8_t *results)
{
    #define   CH_MASK   0x07
    #define   CH_POS    0x04
    #define   VALUE_MASK   0x0F
    #define   VALUE_POS    0

    int i, ch;
    uint8_t data, value = 0;

    memset(results, 0, 8);    // array de resultados en cero (el adc habilita hasta 8 canales)

    for (i = 0; i < ch_num; i++)
    {
        data = buff[i * 2];
        ch = ( data >> CH_POS ) & CH_MASK;
        if (ch != i)
          return -1;    // error en el formato de los datos
    
        value = ( data >> VALUE_POS ) & VALUE_MASK;
        results[i] = value > 8 ? 1 : 0;
    }
    return ch;
}



/*
[PARAMS]    presion_dedos
            orientación

*/

Trama_Agarre agarre(uint8_t *buff){
    Trama_Agarre agarre;
    uint8_t dedos[6] = {0};
    ad7298_parse(buff, 6, dedos);
    #ifdef LAPIZ_TRI

    if (dedos[0] && dedos[4] && dedos[5])
      return Trama_Agarre_TRIDIGITAL;
    else if ( dedos[0] && !dedos[4]  && !dedos[5] ||
              !dedos[0] && dedos[4]  && !dedos[5] ||
              !dedos[0] && !dedos[4] && dedos[5]   )
      return Trama_Agarre_DIGITAL;
    else 
      return Trama_Agarre_MODAL; 

    #elif defined (LAPIZ_HEX)
    if (dedos[0] && !dedos[1] && dedos[2] && !dedos[3] && dedos[4] && !dedos[5]  ||
        !dedos[0] && dedos[1] && !dedos[2] && dedos[3] && !dedos[4] && dedos[5] )
      return Trama_Agarre_TRIDIGITAL;
    else if ( dedos[0] && !dedos[1] && !dedos[2] && dedos[3] && !dedos[4] && !dedos[5]  ||
              !dedos[0] && dedos[1] && !dedos[2] && !dedos[3] && dedos[4] && dedos[5]   ||
              !dedos[0] && !dedos[1] && dedos[2] && !dedos[3] && !dedos[4] && dedos[5]  )
      return Trama_Agarre_DIGITAL;
    else 
      return Trama_Agarre_MODAL;

    #endif
}

Trama_Presion presion(int16_t val){
    if (val <= 8)
      return Trama_Presion_HIPOTONICO;
    else if (val > 8 && val < 16)
      return Trama_Presion_NORMAL;
    else
      return Trama_Presion_HIPERTONICO;   
}

Trama_Direccionalidad direccionalidad(float pitch, float roll){
    if( pitch > 40 || pitch < -40 || roll < 50 || roll > 130)
      return Trama_Direccionalidad_MALA;
    else 
      return Trama_Direccionalidad_BUENA;
}

Trama_Animo animo(uint8_t val){}

Trama_Movimiento movimiento(void * info){}

uint32_t bateria(int16_t val){
    
}

int mayor_elemento(uint8_t *array, int size){    //devuelve el indice del número mayor en el array
    int indice = 0;

    for (int i = 1; i < size; i++){
        if (array[i] > array[indice])
            indice = i;
    }
    return indice;
}

Trama evaluar_tramas_actividad(Trama *tramas_actividad, uint8_t cnt_trama){
//    extern Trama *tramas_actividad;
//    extern uint8_t cnt_trama;
    Trama trama_final = Trama_init_default;
    uint8_t presion[3] = {0};
    uint8_t agarre[3] = {0};
    uint8_t direccionalidad[2] = {0};

    for (int i = 0; i < cnt_trama; i++){
        switch (tramas_actividad[i].presion){
            case Trama_Presion_HIPOTONICO:
                presion[Trama_Presion_HIPOTONICO]++;
                break;
            case Trama_Presion_HIPERTONICO:
                presion[Trama_Presion_HIPERTONICO]++;
                break;
            case Trama_Presion_NORMAL:
                presion[Trama_Presion_NORMAL]++;
                break;
            default:
                break;
        } //fin switch

        switch (tramas_actividad[i].agarre){
            case Trama_Agarre_MODAL:
                agarre[Trama_Agarre_MODAL]++;
                break;
            case Trama_Agarre_DIGITAL:
                agarre[Trama_Agarre_DIGITAL]++;
                break;
            case Trama_Agarre_TRIDIGITAL:
                agarre[Trama_Agarre_TRIDIGITAL]++;
                break;
            default:
                break;
        }

        switch (tramas_actividad[i].direccionalidad){
            case Trama_Direccionalidad_MALA:
                direccionalidad[Trama_Direccionalidad_MALA]++;
                break;
            case Trama_Direccionalidad_BUENA:
                direccionalidad[Trama_Direccionalidad_BUENA]++;
                break;
            default:
                break;
        }

        // AGARRE
    }

    trama_final.presion = mayor_elemento(presion, sizeof(presion)/sizeof(presion[0]));
    trama_final.agarre = mayor_elemento(agarre, sizeof(agarre)/sizeof(agarre[0]));
    trama_final.direccionalidad = mayor_elemento(direccionalidad, sizeof(direccionalidad)/sizeof(direccionalidad[0]));
    
    return trama_final;

    
}
