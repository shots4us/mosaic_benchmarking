#include "excelexport.h"

#include <QtGui>
#include <QAxObject>
#include <QDebug>

#include <QString>

using namespace std;

ExcelExport::ExcelExport()
{
}

ExcelExport::~ExcelExport()
{

}

// QT Excel syntax: http://www.lxway.com/626169141.htm

bool ExcelExport::createExcelFile(const string file_name, const ExportDictionary &xl, const bool visible)
{
    try
    {
        if (xl.m_data.size() == 0)
        {
            cout << "The data repository is empty. Excel file generation abort !!" << endl;
            return false;
        }

        if (xl.m_header.size() == 0)
        {
            cout << "The data repository has no header data !!" << endl;
        }

        QAxObject* p_excel = new QAxObject("Excel.Application", 0);
        p_excel->dynamicCall("SetDisplayAlerts(bool)", true);
        p_excel->setProperty("Caption", "Image compositing report");
        p_excel->setProperty("Visible", visible );
        p_excel->dynamicCall("SetVisible(bool)", visible);

        QAxObject *p_workbooks = p_excel->querySubObject( "Workbooks" );
        // p_workbooks->dynamicCall("Open (const QString&)", QString("c:/test.xls"));
        // QAxObject *p_workbook = p_excel.querySubObject("ActiveWorkBook");

        QAxObject *p_workbook = p_workbooks->querySubObject("Add()");

        QAxObject *worksheets = p_workbook->querySubObject("WorkSheets");
        int intCount = worksheets->property("Count").toInt();

        QAxObject *p_sheets = p_workbook->querySubObject("Worksheets");
        QAxObject *p_sheet_1 = p_sheets->querySubObject("Item(int)", 1);
        p_sheet_1->setProperty("Name", "Interseam");

        // Filling the Excel sheet
        for (int c = 0; c < xl.m_header.size(); c++)
        {
            QAxObject* p_cell = p_sheet_1->querySubObject("Cells(Int, Int)", SheetHeaderRow, c + 1);
            p_cell->setProperty("HorizontalAlignment", -4108);
            p_cell->querySubObject("Font")->setProperty("Bold", true);
            p_cell->querySubObject("Columns(1)")->setProperty("ColumnWidth", 20);
            p_cell->dynamicCall("SetValue(String)", xl.m_header[c].c_str());
        }

        for (int k = 0; k < xl.m_data.size(); k++)
        {
            ExportSheet xl_sheet;
            xl_sheet = xl.m_data[k];

            for (int c = 0; c < xl_sheet.size(); c++)
            {
                ExportColumn xl_c = xl.m_data[k][c];

                for (int r = 0; r < xl_c.size(); r++)
                {
                    QAxObject* p_cell = p_sheet_1->querySubObject("Cells(int,int)", SheetHeaderRow + r + 1, c + 1);
                    p_cell->dynamicCall("SetValue(double)", xl_c[r]);
                }
            }
        }

        // Add chart to the Excel Sheet
        // p_sheet_1->dynamicCall("AddChartAutoFormat ", QVariant("Interseam"));
        p_sheet_1->dynamicCall("xlChartElementPositionAutomatic", QVariant("true"));
        // p_sheet_1->dynamicCall("xlChartElementPositionCustom", QVariant("true"));

        QAxObject *p_charts = p_workbook->querySubObject("Charts");
        QAxObject *p_chart_1 = p_charts->querySubObject("Add");

        p_chart_1->setProperty("Name", "Interseam Chart");
        p_chart_1->setProperty("ChartType", 74);

        // The "PlotBy Excel property is automatically seleced upon the longed dimension size
        // This property must be forced to xlColumn
        // 1 = VBA xlRows, 2 = VBA xlColumns
        p_chart_1->setProperty("PlotBy", 2);
        p_chart_1->setProperty("ChartTitle", QString("Distortions"));

        QAxObject *p_serie_1 = p_chart_1->querySubObject("SeriesCollection(int)", 1);
        QAxObject *xvalues = p_sheet_1->querySubObject("Range(A1:O1)");
        QAxObject *yvalues = p_sheet_1->querySubObject("Range(A2:O2)");

        // Sets the Header metadata to the chart
        p_serie_1->setProperty("XValues", xvalues->asVariant());
        p_serie_1->setProperty("Values", yvalues->asVariant());

        // p_workbook->querySubObject("SaveAs (const QString&)", QVariant(file_name.c_str()));

        //  p_sheet_1->dynamicCall("xlChartElementPositionCustom", QVariant("true"));

        // Chart xlChart = (Excel.Chart)ThisWorkbook.ChartsAdd(Type.Missing, xlSheet, Type.Missing, Type.Missing);

        // SetShowChartTipNames
        // SetShowChartTipValues

        // SetDefaultChart

        return true;

        for (int c = 0; c < xl.m_header.size(); c++)
        {
            QAxObject* p_cell = p_sheet_1->querySubObject("Cells(Int, Int)", SheetHeaderRow, c + 1);
            p_cell->setProperty(xl.m_header[c].c_str(), "");
        }

        for (int k = 0; k < xl.m_data.size(); k++)
        {
            ExportSheet xl_r = xl.m_data[k];

            for (int r = 0; r < xl_r.size(); r++)
            {
                ExportColumn xl_c = xl.m_data[k][r];

                for (int c = 0; c < xl_c.size(); c++)
                {
                    QAxObject* p_cell = p_sheet_1->querySubObject("Cells(Int, Int)", SheetHeaderRow, c);
                    p_cell->setProperty("Val", 12); //xl_c[c]);
                }
            }
        }

        p_sheet_1->dynamicCall("SetDefaultFilePath", QString("D:/PRJ/Benchmark/Excel/"));
        p_excel->setProperty("DisplayAlerts", 0);
        p_workbook->querySubObject("SaveAs (const QString&)", QString("D:/PRJBenchmark/Excel/file1.xls"));
        p_excel->setProperty("DisplayAlerts", 1);
        p_workbook->dynamicCall("Close (Boolean)", true);

        // close the excel file
        p_excel->dynamicCall("Quit (void)");

    }
    catch (exception e)
    {
        cout << e.what() << endl;
    }
    return true;
}


