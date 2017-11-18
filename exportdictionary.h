#ifndef EXPORTDICTIONARY
#define EXPORTDICTIONARY

#include "global.h"
#include "exportdictionary.h"
#include <iostream>
#include <vector>

using namespace std;

typedef vector<string> ExportHeader;
typedef vector<double> ExportTab;
typedef vector<ExportTab> ExportColumn;             // Different aggregate metrics
typedef vector<ExportColumn> ExcelRow;              // Different agregate distortion steps
typedef vector<ExportColumn> ExportSheet;           // Different aggregate image sets

typedef struct
{
    ExportHeader m_header;
    vector <ExportSheet> m_data;
} ExportDictionary;

#endif // EXPORTDICTIONARY

