using System;
using System.Collections.Generic;
using System.Text;

using VRage.Utils;

namespace ttdtwm
{
    // Technically a struct, but C# doesnt' allow to modify fields of a struct which is implemented as property
    class solver_entry
    {
        public float x, y, max_value, result;
    };

    class simplex_solver
    {
        const double GUARD_VALUE = 1.0E-6;

        private double[][] _tableau;    // ModAPI doesn't like true multidimesional arrays
        private int  _current_width = 0;

        public List<solver_entry> items { get; private set; }

        private void log_tableau(int height, int width)
        {
            StringBuilder row = new StringBuilder();

            MyLog.Default.WriteLine("");
            for (int cur_row = 0; cur_row < height; ++cur_row)
            {
                row.Clear();
                for (int cur_colum = 0; cur_colum < width; ++cur_colum)
                {
                    char non_zero_symbol = (_tableau[cur_row][cur_colum] != 0.0 && Math.Abs(_tableau[cur_row][cur_colum]) < GUARD_VALUE) ? '*' : ' ';
                    row.AppendFormat("{0,10:F2}{1}", _tableau[cur_row][cur_colum], non_zero_symbol);
                }
                MyLog.Default.WriteLine(row.ToString());
            }
            MyLog.Default.WriteLine("");
        }

        private void fill_tableau(int item_count, int height, int width)
        {
            if (_current_width >= width)
            {
                foreach (var cur_row in _tableau)
                    Array.Clear(cur_row, 0, _current_width);
            }
            else
            {
                _tableau = new double[height][];
                for (int cur_row = 0; cur_row < height; ++cur_row)
                    _tableau[cur_row] = new double[width];
                _current_width = width;
            }

            for (int item_index = 0; item_index < item_count; ++item_index)
            {
                // Centre location constraints
                _tableau[0][item_index] =  items[item_index].x;
                _tableau[1][item_index] =  items[item_index].y;
                _tableau[2][item_index] = -items[item_index].x;
                _tableau[3][item_index] = -items[item_index].y;

                // Force constraints and slack variables
                _tableau[item_index + 4][item_index] = _tableau[item_index + 4][item_index + item_count + 4] = 1.0;
                _tableau[item_index + 4][ width - 1] = items[item_index].max_value;
                _tableau[    height - 1][item_index] = -1.0;
            }

            // Centre location slack variables
            for (int cur_row = 0; cur_row < 4; ++cur_row)
                _tableau[cur_row][cur_row + item_count] = 1.0;
            _tableau[height - 1][width - 2] = 1.0;
        }

        private bool solve(int height, int width)
        {
            while (true)
            {
                //log_tableau(height, width);

                double min_column_value = 0.0;
                int    pivot_column     = -1, cur_column;
                for (cur_column = 0; cur_column < width - 1; ++cur_column)
                {
                    if (min_column_value > _tableau[height - 1][cur_column])
                    {
                        min_column_value = _tableau[height - 1][cur_column];
                        pivot_column     = cur_column;
                    }
                }
                if (pivot_column < 0)
                    return true;

                double min_ratio = double.MaxValue, cur_ratio;
                int    pivot_row = -1, cur_row;
                for (cur_row = 0; cur_row < height - 1; ++cur_row)
                {
                    if (_tableau[cur_row][pivot_column] > 0.0)
                    {
                        cur_ratio = _tableau[cur_row][width - 1] / _tableau[cur_row][pivot_column];
                        if (min_ratio > cur_ratio && cur_ratio >= 0.0)
                        {
                            min_ratio = cur_ratio;
                            pivot_row = cur_row;
                        }
                    }
                }
                if (pivot_row < 0)
                    return false;

                double divider = _tableau[pivot_row][pivot_column], multiplier;
                for (cur_column = 0; cur_column < width; ++cur_column)
                    _tableau[pivot_row][cur_column] /= divider;
                _tableau[pivot_row][pivot_column] = 1.0;
                for (cur_row = 0; cur_row < height; ++cur_row)
                {
                    if (cur_row == pivot_row)
                        continue;
                    multiplier = _tableau[cur_row][pivot_column];
                    for (cur_column = 0; cur_column < width; ++cur_column)
                    {
                        _tableau[cur_row][cur_column] -= multiplier * _tableau[pivot_row][cur_column];
                        
                        if (Math.Abs(_tableau[cur_row][cur_column]) <= GUARD_VALUE)
                            _tableau[cur_row][cur_column] = 0.0;
                    }
                    _tableau[cur_row][pivot_column] = 0.0;
                }
            }
        }

        private void extract_values(int item_count, int height, int width)
        {
            int cur_row;

            for (int item_index = 0; item_index < item_count; ++item_index)
            {
                items[item_index].result = 0.0f;
                for (cur_row = 0; cur_row < height - 1; ++cur_row)
                {
                    if (Math.Abs(_tableau[cur_row][item_index]) > GUARD_VALUE)
                    {
                        items[item_index].result = (float) (_tableau[cur_row][width - 1] / _tableau[cur_row][item_index]);
                        break;
                    }
                }
                for (++cur_row; cur_row < height - 1; ++cur_row)
                {
                    if (Math.Abs(_tableau[cur_row][item_index]) > GUARD_VALUE)
                    {
                        items[item_index].result = 0.0f;
                        break;
                    }
                }
            }
        }

        private bool is_solution_good(int item_count)
        {
            //float sum_x = 0.0f, sum_y = 0.0f;
            float max_ratio = 0.0f;

            for (int cur_item = 0; cur_item < item_count; ++cur_item)
            {
                if (items[cur_item].result < 0.0f || items[cur_item].result > items[cur_item].max_value)
                    return false;
                if (items[cur_item].max_value > 0.0f && max_ratio < items[cur_item].result / items[cur_item].max_value)
                    max_ratio = items[cur_item].result / items[cur_item].max_value;
                //sum_x += items[cur_item].result * items[cur_item].x;
                //sum_y += items[cur_item].result * items[cur_item].y;
            }
            //MyLog.Default.WriteLine(string.Format("TT&DT\t\tsimplex_solver.is_solution_good(): {0}/{1} residual static moment", sum_x, sum_y));
            return max_ratio >= 0.8f;
        }

        public simplex_solver()
        {
            items = new List<solver_entry>();
        }

        public bool calculate_solution(int item_count)
        {
            int tableau_width = 2 * item_count + 6, tableau_height = item_count + 5;

            fill_tableau(item_count, tableau_height, tableau_width);
            if (!solve(tableau_height, tableau_width))
                return false;
            extract_values(item_count, tableau_height, tableau_width);
            return is_solution_good(item_count);
        }
    }
}
