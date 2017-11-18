#ifndef EXCELEXPORT_H
#define EXCELEXPORT_H

#include "exportdictionary.h"
#include <iostream>
#include <ctime>
#include <sstream>
#include <vector>
#include <stdint.h>

using namespace std;

const int SheetHeaderRow = 1;

//typedef vector<string> ExportHeader;
//typedef vector<float> ExcelColumn;                 // Different aggregate metrics
//typedef vector<ExcelColumn> ExcelRow;              // Different agregate distortion steps
//typedef vector<ExcelColumn> ExcelSheet;            // Different aggregate image sets

//typedef struct
//{
//    ExportHeader m_header;
//    vector <ExcelSheet> m_data;
//} ExportDictionary;


class ExcelExport
{
public:
    ExcelExport();
    ~ExcelExport();
    bool createExcelFile(const string file_name, const ExportDictionary &xl, const bool visible);
};

#endif // EXCELEXPORT_H
