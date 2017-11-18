#include <iostream>
#include <ctime>
#include <sstream>
#include <vector>
#include <stdint.h>
#include <string.h>
#include "image_compose.h"
#include "gnuplotexport.h"

#ifdef IS_WIN
#include "windows.h"
#endif

using namespace std;

GnuPlotExport::GnuPlotExport()
{
}

GnuPlotExport::~GnuPlotExport()
{
}

bool GnuPlotExport::exportColumnToCSV(const string file_name,
                                      const ExportDictionary &xl,
                                      const int f_idx,
                                      const int d_idx,
                                      const bool skip_image_0)
{
    FILE *fid = fopen(file_name.data(),"w+");

    if (!fid)
    {
        return false;
    }

    ExportSheet xl_sheet;
    xl_sheet = xl.m_data[0];

    vector<string> row_vct;

    for (int r = 0; r < xl_sheet[f_idx][d_idx].size(); r++)
    {
        row_vct.push_back("");
    }

    ExportTab xl_c = xl_sheet[f_idx][d_idx];

    int r_0 = 0;
    if (skip_image_0 == true)
    {
        r_0++;
    }

    for (int r = r_0; r < xl_c.size(); r++)
    {
        char val[128];
        memset(val, 0, 128);
        string str = "";
        sprintf(val, "%d", r + 1 - r_0);

        str += string(val);
        str += ", ";

        sprintf(val, "%6.6f", xl_c[r]);
        str += string(val);
        str += '\n';

        row_vct[r] += str;
        fprintf(fid, row_vct[r].data());
    }
    fclose(fid);
    return true;
}


bool GnuPlotExport::generateGnuPlot(const string path,
                                    const string plot_file,
                                    const string plot_ext,
                                    const string CSV_file,
                                    const ExportDictionary &xl,
                                    const string x_label,
                                    const string y_label,
                                    const bool is_log_scale)
{  
    const int k = 0;
    ExportSheet xl_sheet;
    xl_sheet = xl.m_data[k];

    int c = 0;
    string gnu_plot_cmd = "plot ";
    char val[64];
    memset(val, 0, 64);
    sprintf(val, "%d", c + 2);

    gnu_plot_cmd += "'";
    gnu_plot_cmd += CSV_file;
    gnu_plot_cmd += "' u 1:" + string(val) + " w l title '" + xl.m_header[0] + "'";

    chdir((path + "Export").c_str());
    FILE *fid = fopen("tmp.gnu", "w+");

    if (!fid)
    {
#ifdef IS_VERBOSE
        cout << "Creation of the tmp.gnu file failed !!" << endl;
#endif
        return false;
    }

    // Prepares the temporary gnuplot script file that gnuplot will process
    string str = "set term pngcairo";
    str += '\n';
    str += "set output '";
    str += plot_file;
    str += '_';
    str += xl.m_header[0];
    str += plot_ext;
    str += "'";
    str += '"\n';
    str += "set style line 1 lt 1";
    str += '"\n';
    str += "set style line 2 lt 3";
    str += '"\n';
    str += "set style line 3 lt 4";
    str += '"\n';
    str += "set style line 4 lt 5";
    str += '\n';
    str += "set xlabel '" + x_label + "' textcolor lt 2";
    str += '"\n';
    str += "set ylabel '" + y_label + "' textcolor lt 2";
    str += '\n';
    //  str += "set yrange [0:10]";
    str += '\n';
    //  str += "set xrange [0:10]";
    str += '\n';
    str += "set grid xtics mxtics"; // draw lines for eachxytics and mxtics
    str += '\n';
    str += "set mxtics 2";          // set the spacing for the mxtics
    str += '\n';
    str += "set grid";              // enable the grid
    str += '\n';
    str += "set grid ytics mytics"; // draw lines for each ytics and mytics
    str += '\n';
    str += "set mytics 2";          // set the spacing for the mytics
    str += '\n';
    str += "set grid";              // enable the grid
    str += '\n';

    if (is_log_scale == true)
    {
        str += "set logscale y 2";
    }
    str += '\n';
    str += gnu_plot_cmd;
    str += '\n';
    str += "exit";

    // Saves the temporary gnuplot script file
    fprintf(fid, str.data());
    fclose(fid);

    // Execute gnuplot and process the script file
#ifdef IS_WIN
    system("gnuplot.exe tmp.gnu");
#endif

#ifdef IS_LINUX
    system("gnuplot tmp.gnu");
#endif

    if (remove("tmp.gnu") != 0)
    {
#ifdef IS_VERBOSE
        cout << "The file tmp.gnu could not be removed !!" << endl;
#endif
        return false;
    }
    return true;
}

bool GnuPlotExport::exportMergedColumnsToCSV(const string file_name,
                                             const ExportDictionary &xl,
                                             const int d_idx,
                                             const bool skip_image_0)
{
    FILE *fid = fopen(file_name.data(),"w+");

    if (!fid)
    {
        return false;
    }

    ExportSheet xl_sheet;
    xl_sheet = xl.m_data[0];

    vector<string> row_vct;

    for (int r = 0; r < xl_sheet[0][0].size(); r++)
    {
        row_vct.push_back("");
    }

    for (int c = 0; c < xl_sheet.size(); c++)
    {
        // Adds the first column as counter required by GnuPlot
        ExportColumn xl_c = xl_sheet[c];

        int r_0 = 0;
        if (skip_image_0 == true)
        {
            r_0++;
        }

        for (int r = r_0; r < xl_c[d_idx].size(); r++)
        {
            char val[128];
            memset(val, 0, 128);
            string str = "";

            if (c == 0)
            {
                if (skip_image_0 == false)
                {
                    sprintf(val, "%d", r + 1 - r_0);
                }
                else
                {
                    sprintf(val, "%d", r);
                }

                str += string(val);
                str += ", ";
            }
            sprintf(val, "%6.6f", xl_c[d_idx][r]);
            str += string(val);
            if (c < xl_sheet.size() - 1)
            {
                str += ", ";
            }
            else
            {
                str += '\n';
            }
            row_vct[r] += str;
        }
    }

    for (int i = 0; i < row_vct.size(); i++)
    {
        fprintf(fid, row_vct[i].data());
    }
    fclose(fid);
    return true;
}

bool GnuPlotExport::generateMergedGnuPlot(const string path,
                                          const string plot_file,
                                          const string plot_ext,
                                          const string CSV_file,
                                          const ExportDictionary &xl,
                                          const string x_label,
                                          const string y_label,
                                          const bool is_log_scale)
{
    string gnu_plot_cmd = "plot ";

    ExportSheet xl_sheet;
    xl_sheet = xl.m_data[0];

    for (int c = 0; c < xl_sheet.size(); c++)
    {
        char val[64];
        memset(val, 0, 64);
        sprintf(val, "%d", c + 2);
        gnu_plot_cmd += "'";
        gnu_plot_cmd += CSV_file;
        gnu_plot_cmd += "' u 1:" + string(val) + " w l title '" + xl.m_header[c];

        if (c < xl_sheet.size() - 1)
        {
            gnu_plot_cmd += "', ";
        }
    }

    chdir((path + "Export").c_str());
    FILE *fid = fopen("tmp.gnu", "w+");

    if (!fid)
    {
#ifdef IS_VERBOSE
        cout << "Creation of the tmp.gnu file failed !!" << endl;
#endif
        return false;
    }

    string str = "set term pngcairo";
    str += '\n';
    str += "set output '";
    str += plot_file;
    str += plot_ext;
    str += '"\n';
    str += "set style line 1 lt 1";
    str += '"\n';
    str += "set style line 2 lt 3";
    str += '"\n';
    str += "set style line 3 lt 4";
    str += '"\n';
    str += "set style line 4 lt 5";
    str += '\n';
    str += "set xlabel '" + x_label + "' textcolor lt 2";
    str += '"\n';
    str += "set ylabel '" + y_label + "' textcolor lt 2";
    str += '\n';
    str += "set grid xtics mxtics"; // draw lines for eachxytics and mxtics
    str += '\n';
    str += "set mxtics 2";          // set the spacing for the mxtics
    str += '\n';
    str += "set grid";              // enable the grid
    str += '\n';
    str += "set grid ytics mytics"; // draw lines for each ytics and mytics
    str += '\n';
    str += "set mytics 2";          // set the spacing for the mytics
    str += '\n';
    str += "set grid";              // enable the grid

    if (is_log_scale == true)
    {
        str += "set logscale y 2";
    }
    str += '\n';
    str += gnu_plot_cmd;
    str += '"\n';
    str += "exit";

    fprintf(fid, str.data());
    fclose(fid);

#ifdef IS_WIN
    system("gnuplot.exe tmp.gnu");
#endif

#ifdef IS_LINUX
    system("gnuplot tmp.gnu");
#endif

    if (remove("tmp.gnu") != 0)
    {
#ifdef IS_VERBOSE
        cout << "The file tmp.gnu could not be removed !!" << endl;
#endif
        return false;
    }
    return true;
}
