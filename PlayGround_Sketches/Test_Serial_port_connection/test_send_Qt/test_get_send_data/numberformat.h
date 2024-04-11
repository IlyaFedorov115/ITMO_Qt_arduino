#ifndef NUMBERFORMAT_H
#define NUMBERFORMAT_H

#include <QMap>
#include <QString>

enum NumberFormat
{
    NumberFormat_uint8,
    NumberFormat_uint16,
    NumberFormat_uint32,
    NumberFormat_int8,
    NumberFormat_int16,
    NumberFormat_int32,
    NumberFormat_float,
    NumberFormat_double,
    NumberFormat_INVALID ///< used for error cases
};

/// Convert `NumberFormat` to string for representation
QString numberFormatToStr(NumberFormat nf);

/// Convert string to `NumberFormat`
NumberFormat strToNumberFormat(QString str);



#endif // NUMBERFORMAT_H
