#include "numberformat.h"

QMap<NumberFormat, QString> mapping({
        {NumberFormat_uint8, "uint8"},
        {NumberFormat_uint16, "uint16"},
        {NumberFormat_uint32, "uint32"},
        {NumberFormat_int8, "int8"},
        {NumberFormat_int16, "int16"},
        {NumberFormat_int32, "int32"},
        {NumberFormat_float, "float"},
        {NumberFormat_double, "double"}
    });

QString numberFormatToStr(NumberFormat nf)
{
    return mapping.value(nf);
}

NumberFormat strToNumberFormat(QString str)
{
    return mapping.key(str, NumberFormat_INVALID);
}
