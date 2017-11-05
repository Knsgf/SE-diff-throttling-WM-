using System;
using System.Collections.Generic;
using System.Text;

using VRage.Utils;

namespace ttdtwm
{
    // Technically a struct, but C# doesnt' allow to modify fields of a struct which is implemented as property
    sealed class solver_entry
    {
        public float x, y, max_value, result;
    };

    sealed class simplex_solver
    {
        const double GUARD_VALUE = 1.0E-6;

        private double[][] _tableau;    // ModAPI doesn't like true multidimesional arrays
        private int        _current_width = 0;

        public List<solver_entry> items { get; private set; }

        private void log_tableau(int height, int width, int pivot_row, int pivot_column)
        {
            StringBuilder row = new StringBuilder();

            MyLog.Default.WriteLine("");
            for (int cur_row = 0; cur_row < height; ++cur_row)
            {
                row.Clear();
                for (int cur_colum = 0; cur_colum < width; ++cur_colum)
                {
                    char non_zero_symbol = (_tableau[cur_row][cur_colum] != 0.0 && Math.Abs(_tableau[cur_row][cur_colum]) < GUARD_VALUE) ? '*' : ' ';

                    if (non_zero_symbol == ' ' && (cur_row == pivot_row || cur_colum == pivot_column))
                        non_zero_symbol = '#';
                    row.AppendFormat("{0,10:G3}{1}", _tableau[cur_row][cur_colum], non_zero_symbol);
                }
                MyLog.Default.WriteLine(row.ToString());
            }
            MyLog.Default.WriteLine("");
        }

        private void fill_tableau(int item_count, int height, int width)
        {
            if (_current_width == width)
            {
                foreach (var cur_row in _tableau)
                    Array.Clear(cur_row, 0, width);
            }
            else
            {
                _tableau = new double[height][];
                for (int cur_row = 0; cur_row < height; ++cur_row)
                    _tableau[cur_row] = new double[width];
                _current_width = width;
            }

            int RHS_column = width - 1, objective_row1 = height - 1, objective_row2 = height - 2;
            for (int item_index = 0; item_index < item_count; ++item_index)
            {
                double x = items[item_index].x, y = items[item_index].y;

                // Centre location constraints
                _tableau[0][item_index] = x;
                _tableau[1][item_index] = y;

                // Force constraints and slack variables
                double[] force_row = _tableau[2 + item_index];
                force_row[item_index] = force_row[item_index + item_count] = 1.0;
                force_row[RHS_column] = items[item_index].max_value;

                // Primary and auxiliary objective function coefficients
                _tableau[objective_row2][item_index] = -1.0;
                _tableau[objective_row1][item_index] = -x - y;
            }

            // Artificial variables
            _tableau[0][RHS_column - 2] = _tableau[1][RHS_column - 1] = 1.0;
        }

        private bool solve(ref int height, ref int width)
        {
            const uint MAX_ITERATIONS = 500;

            uint iterations = 0;
            int  cur_objective_row = height - 1, objective_row2 = height - 2, cur_RHS_column = width - 1, last_row = -1, last_column = -1;
            bool a1_is_basic = true, a2_is_basic = true;

            while (true)
            {
                if (engine_control_unit.CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine(string.Format("Iteration #{0}/{1}", iterations, MAX_ITERATIONS));

                double[] objective_row_ref = _tableau[cur_objective_row];
                double   min_column_value  = 0.0;
                int      pivot_column      = -1, cur_column, cur_row;
                for (cur_column = last_column + 1; cur_column < cur_RHS_column; ++cur_column)
                {
                    if (min_column_value > objective_row_ref[cur_column])
                    {
                        min_column_value = objective_row_ref[cur_column];
                        pivot_column     = cur_column;
                    }
                }
                for (cur_column = 0; cur_column <= last_column; ++cur_column)
                {
                    if (min_column_value > objective_row_ref[cur_column])
                    {
                        min_column_value = objective_row_ref[cur_column];
                        pivot_column     = cur_column;
                    }
                }
                if (pivot_column < 0 || ++iterations >= MAX_ITERATIONS)
                {
                    if (engine_control_unit.CALIBRATION_DEBUG)
                        log_tableau(cur_objective_row + 1, cur_RHS_column + 1, -1, -1);
                    if (cur_objective_row == height - 1)
                    {
                        if (a1_is_basic || a2_is_basic)
                            return false;
                        int old_RHS_column = cur_RHS_column;
                        --cur_objective_row;
                        cur_RHS_column -= 2;
                        for (cur_row = 0; cur_row <= cur_objective_row; ++cur_row)
                            _tableau[cur_row][cur_RHS_column] = _tableau[cur_row][old_RHS_column];
                        last_row = last_column = -1;
                        continue;
                    }
                    return true;
                }

                double[] cur_row_ref;
                double   min_ratio = double.MaxValue, cur_ratio;
                int      pivot_row = -1;
                for (cur_row = last_row + 1; cur_row < objective_row2; ++cur_row)
                {
                    cur_row_ref = _tableau[cur_row];
                    if (cur_row_ref[pivot_column] > 0.0)
                    {
                        cur_ratio = cur_row_ref[cur_RHS_column] / cur_row_ref[pivot_column];
                        if (min_ratio > cur_ratio && cur_ratio >= 0.0)
                        {
                            min_ratio = cur_ratio;
                            pivot_row = cur_row;
                        }
                    }
                }
                for (cur_row = 0; cur_row <= last_row; ++cur_row)
                {
                    cur_row_ref = _tableau[cur_row];
                    if (cur_row_ref[pivot_column] > 0.0)
                    {
                        cur_ratio = cur_row_ref[cur_RHS_column] / cur_row_ref[pivot_column];
                        if (min_ratio > cur_ratio && cur_ratio >= 0.0)
                        {
                            min_ratio = cur_ratio;
                            pivot_row = cur_row;
                        }
                    }
                }
                if (pivot_row < 0)
                {
                    if (engine_control_unit.CALIBRATION_DEBUG)
                        log_tableau(cur_objective_row + 1, cur_RHS_column + 1, -1, pivot_column);
                    return false;
                }
                else if (pivot_row == 0)
                    a1_is_basic = false;
                else if (pivot_row == 1)
                    a2_is_basic = false;
                last_row    = pivot_row;
                last_column = pivot_column;

                if (engine_control_unit.CALIBRATION_DEBUG)
                    log_tableau(cur_objective_row + 1, cur_RHS_column + 1, pivot_row, pivot_column);

                double[] pivot_row_ref = _tableau[pivot_row];
                double   divider       = pivot_row_ref[pivot_column], multiplier;
                for (cur_column = 0; cur_column <= cur_RHS_column; ++cur_column)
                    pivot_row_ref[cur_column] /= divider;
                pivot_row_ref[pivot_column] = 1.0;
                for (cur_row = 0; cur_row <= cur_objective_row; ++cur_row)
                {
                    if (cur_row == pivot_row)
                        continue;
                    cur_row_ref = _tableau[cur_row];
                    multiplier  = cur_row_ref[pivot_column];
                    for (cur_column = 0; cur_column < pivot_column; ++cur_column)
                    {
                        cur_row_ref[cur_column] -= multiplier * pivot_row_ref[cur_column];
                        if (Math.Abs(cur_row_ref[cur_column]) <= GUARD_VALUE)
                            cur_row_ref[cur_column] = 0.0;
                    }
                    cur_row_ref[pivot_column] = 0.0;
                    for (cur_column = pivot_column + 1; cur_column <= cur_RHS_column; ++cur_column)
                    {
                        cur_row_ref[cur_column] -= multiplier * pivot_row_ref[cur_column];
                        if (Math.Abs(cur_row_ref[cur_column]) <= GUARD_VALUE)
                            cur_row_ref[cur_column] = 0.0;
                    }
                }
            }
        }

        private void extract_values(int item_count, int height, int width)
        {
            int cur_row, objective_row = height - 2, RHS_column = width - 3;

            for (int item_index = 0; item_index < item_count; ++item_index)
            {
                items[item_index].result = 0.0f;
                for (cur_row = 0; cur_row <= objective_row; ++cur_row)
                {
                    if (Math.Abs(_tableau[cur_row][item_index]) > GUARD_VALUE)
                    {
                        items[item_index].result = (float) (_tableau[cur_row][RHS_column] / _tableau[cur_row][item_index]);
                        if (items[item_index].result < 0.0f)
                            items[item_index].result = 0.0f;
                        else if (items[item_index].result > items[item_index].max_value)
                            items[item_index].result = items[item_index].max_value;
                        break;
                    }
                }
                for (++cur_row; cur_row <= objective_row; ++cur_row)
                {
                    if (Math.Abs(_tableau[cur_row][item_index]) > GUARD_VALUE)
                    {
                        items[item_index].result = 0.0f;
                        break;
                    }
                }
                if (engine_control_unit.CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine(string.Format("TT&DT\t\tsimplex_solver.extract_values(): [{0}] = {1}", item_index, items[item_index].result));
            }
        }

        private bool is_solution_good(int item_count)
        {
            float sum_x = 0.0f, sum_y = 0.0f;
            float max_ratio = 0.0f;

            for (int cur_item = 0; cur_item < item_count; ++cur_item)
            {
                if (items[cur_item].max_value > 0.0f && max_ratio < items[cur_item].result / items[cur_item].max_value)
                    max_ratio = items[cur_item].result / items[cur_item].max_value;
                if (engine_control_unit.CALIBRATION_DEBUG)
                {
                    sum_x += items[cur_item].result * items[cur_item].x;
                    sum_y += items[cur_item].result * items[cur_item].y;
                }
            }
            if (engine_control_unit.CALIBRATION_DEBUG)
                MyLog.Default.WriteLine(string.Format("TT&DT\t\tsimplex_solver.is_solution_good(): {0}/{1} residual static moment; max = {2}", sum_x, sum_y, max_ratio));
            return max_ratio >= 0.8f;
        }

        public simplex_solver()
        {
            items = new List<solver_entry>();
        }

        public bool calculate_solution(int item_count)
        {
            int tableau_width = 2 * item_count + 3, tableau_height = item_count + 4;

            fill_tableau(item_count, tableau_height, tableau_width);
            if (!solve(ref tableau_height, ref tableau_width))
                return false;
            extract_values(item_count, tableau_height, tableau_width);
            return is_solution_good(item_count);
        }
    }
}
