# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: datosLapiz.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x10\x64\x61tosLapiz.proto\"\xd5\x02\n\x05Trama\x12\x11\n\tbat_lapiz\x18\x01 \x01(\r\x12\x13\n\x0b\x62\x61t_pulsera\x18\x02 \x01(\r\x12%\n\nprehension\x18\x03 \x01(\x0e\x32\x11.Trama.Prehension\x12\'\n\x0binclinacion\x18\x04 \x01(\x0e\x32\x12.Trama.Inclinacion\x12\x15\n\x02hr\x18\x05 \x01(\x0e\x32\t.Trama.HR\x12!\n\x08pronosup\x18\x06 \x01(\x0e\x32\x0f.Trama.Pronosup\"4\n\nPrehension\x12\t\n\x05MODAL\x10\x00\x12\x0b\n\x07\x44IGITAL\x10\x01\x12\x0e\n\nTRIDIGITAL\x10\x02\"\"\n\x0bInclinacion\x12\x08\n\x04MALA\x10\x00\x12\t\n\x05\x42UENA\x10\x01\"\x1d\n\x02HR\x12\x0b\n\x07\x41NORMAL\x10\x00\x12\n\n\x06NORMAL\x10\x01\"!\n\x08Pronosup\x12\n\n\x06SUPINO\x10\x00\x12\t\n\x05PRONO\x10\x01\"\xab\x01\n\nDatosCrudo\x12\x15\n\rDCprehensionA\x18\x01 \x02(\x0c\x12\x15\n\rDCprehensionB\x18\x02 \x02(\x0c\x12\x15\n\rDCprehensionC\x18\x03 \x02(\x0c\x12\x16\n\x0e\x44\x43inclinacionX\x18\x04 \x02(\x0c\x12\x16\n\x0e\x44\x43inclinacionY\x18\x05 \x02(\x0c\x12\x13\n\x0b\x44\x43pronosupX\x18\x06 \x02(\x0c\x12\x13\n\x0b\x44\x43pronosupY\x18\x07 \x02(\x0c')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'datosLapiz_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _globals['_TRAMA']._serialized_start=21
  _globals['_TRAMA']._serialized_end=362
  _globals['_TRAMA_PREHENSION']._serialized_start=208
  _globals['_TRAMA_PREHENSION']._serialized_end=260
  _globals['_TRAMA_INCLINACION']._serialized_start=262
  _globals['_TRAMA_INCLINACION']._serialized_end=296
  _globals['_TRAMA_HR']._serialized_start=298
  _globals['_TRAMA_HR']._serialized_end=327
  _globals['_TRAMA_PRONOSUP']._serialized_start=329
  _globals['_TRAMA_PRONOSUP']._serialized_end=362
  _globals['_DATOSCRUDO']._serialized_start=365
  _globals['_DATOSCRUDO']._serialized_end=536
# @@protoc_insertion_point(module_scope)
