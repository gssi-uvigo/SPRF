import serial
import time
import datosLapiz_pb2 as dpb



#Guardar valores en la trama
datos = dpb.Trama()
datos.prehension = dpb.Trama.Prehension.Value('TRIDIGITAL')
datos.inclinacion = dpb.Trama.Inclinacion.Value('BUENA')

#Serializar en un archivo
with open('out.bin', 'wb') as f:
    f.write(datos.SerializeToString())

#Deserializar desde el archivo
with open('out.bin', 'rb') as f:
    read_datos = dpb.Trama()
    read_datos.ParseFromString(f.read())

#Imprimir valores decodificados     
print('Inclinacion: {}'.format(read_datos.inclinacion))
print('Prehension: {}'.format(read_datos.prehension))

