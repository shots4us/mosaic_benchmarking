#ifndef GNUPLOTEXPORT_H
#define GNUPLOTEXPORT_H

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#ifdef IS_WIN
#include <direct.h>
#endif

#include <iostream>
#include <ctime>
#include <sstream>
#include <vector>
#include <stdint.h>

using namespace std;

#include "exportdictionary.h"

class GnuPlotExport
{
public:
    GnuPlotExport();
    ~GnuPlotExport();

    bool exportColumnToCSV(const string file_name,
                           const ExportDictionary &xl,
                           const int f_idx,
                           const int d_idx,
                           const bool skip_image_0);

    bool exportMergedColumnsToCSV(const string file_name,
                                  const ExportDictionary &xl,
                                  const int d_idx,
                                  const bool skip_image_0);

    bool generateGnuPlot(const string path,
                         const string plot_file,
                         const string plot_ext,
                         const string CSV_file,
                         const ExportDictionary &xl,
                         const string x_label,
                         const string y_label, const bool is_log_scale);

    bool generateMergedGnuPlot(const string path,
                               const string plot_file,
                               const string plot_ext,
                               const string CSV_file,
                               const ExportDictionary &xl,
                               const string x_label,
                               const string y_label, const bool is_log_scale);
};

#endif // GNUPLOTEXPORT_H
