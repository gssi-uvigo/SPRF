/*    MODELO DE LAPIZ   */
#define LAPIZ_TRI   

#include "indicadores.pb.h"


Trama_Agarre agarre(uint8_t *buff);
Trama_Presion presion(int16_t val);
Trama_Direccionalidad direccionalidad(float pitch, float roll);

Trama evaluar_tramas_actividad(Trama *, uint8_t);
