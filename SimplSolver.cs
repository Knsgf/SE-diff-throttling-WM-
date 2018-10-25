using System;
using System.Text;
using System.Collections.Generic;

using VRage.Utils;

namespace ttdtwm
{
    // Technically a struct, but C# doesnt' allow to modify fields of a struct which is implemented as property
    sealed class solver_entry
    {
        public float x, y, max_value, result;
    };

    sealed class revised_simplex_solver
    {
        public const double EPSILON = 1.0E-8;

        private int           _last_item_count = 0;
        private int[]         _vB = null, _vN = null, _art_var_pos = new int[2];
        private sparse_matrix _E, _cB, _cN, _B, _N, _B_inv;
        private dense_matrix  _b, _z, _z_c;
        private sparse_matrix _entering_column;
        private dense_matrix  _result, _dividers, _total, _coeffs;

        public List<solver_entry> items { get; private set; }

        private bool is_solution_good(int item_count)
        {
            float max_ratio = 0.0f;

            if (!engine_control_unit.CALIBRATION_DEBUG)
            {
                for (int cur_item = 0; cur_item < item_count; ++cur_item)
                {
                    if (items[cur_item].max_value > 0.0f && max_ratio < items[cur_item].result / items[cur_item].max_value)
                        max_ratio = items[cur_item].result / items[cur_item].max_value;
                }
                return max_ratio >= 0.8f;
            }

            float moment_x = 0.0f, moment_y = 0.0f, force_x = float.Epsilon, force_y = float.Epsilon;

            for (int cur_item = 0; cur_item < item_count; ++cur_item)
            {
                MyLog.Default.WriteLine(string.Format("#{0} = {1}/{2}", cur_item, items[cur_item].result, items[cur_item].max_value));
                if (items[cur_item].max_value > 0.0f && max_ratio < items[cur_item].result / items[cur_item].max_value)
                    max_ratio = items[cur_item].result / items[cur_item].max_value;
                moment_x += items[cur_item].result * items[cur_item].x;
                moment_y += items[cur_item].result * items[cur_item].y;
                force_x  += items[cur_item].result;
                force_y  += items[cur_item].result;
            }
            MyLog.Default.WriteLine(string.Format("TT&DT\t\tsimplex_solver.is_solution_good(): {0}/{1} residual offset; max = {2}", moment_x / force_x, moment_y / force_y, max_ratio));
            return max_ratio >= 0.8f;
        }

        static private void log_var(string prefix, int var, int item_count)
        {
            MyLog.Default.WriteLine(string.Format("{0} #{1}/{2}", prefix, var + 1, item_count));
        }

        private void fill_matrices(int item_count)
        {
            int num_vars = item_count + 2;

            if (item_count != _last_item_count)
            {
                _vB = new int[  num_vars];
                _vN = new int[item_count];
                _last_item_count = item_count;
            }

            _E.set_to_identity(num_vars);
            _cB.clear(1,   num_vars);
            _cN.clear(1, item_count);
            _B.set_to_identity(num_vars);
            _B_inv.clear(num_vars);
            _N.clear(num_vars, item_count);
            _b.clear(num_vars, 1);
            //_B_inv.set_to_identity(num_vars);
            _z.clear  (1,   num_vars);
            _z_c.clear(1, item_count);

            _result.clear         (num_vars, 1);
            _entering_column.clear(num_vars, 1);
            _dividers.clear       (num_vars, 1);

            sparse_row_ref B0_ref = _B[0], B1_ref = _B[1];
            int            cur_index;
            for (int cur_item = 0; cur_item < item_count; ++cur_item)
            {
                cur_index = cur_item + 2;
                _vB   [cur_index] = cur_item;
                _vN   [cur_item ] = cur_item + num_vars;
                B0_ref[cur_index] = items[cur_item].x;
                B1_ref[cur_index] = items[cur_item].y;
                _N[cur_index][cur_item] = 1.0;
                _b[cur_index][       0] = items[cur_item].max_value;
            }
            _vB[0] = item_count;
            _vB[1] = item_count + 1;
        }

        private bool perform_pivot(int num_vars, int entering_var, int leaving_var, int iterations)
        {
            int temp          = _vN[entering_var];
            _vN[entering_var] = _vB[leaving_var];
            _vB[leaving_var]  = temp;
            sparse_matrix.exchange_columns(_cN, entering_var, _cB, leaving_var);
            sparse_matrix.exchange_columns( _N, entering_var,  _B, leaving_var);
            if ((iterations & 0xF) == 0)
            {
                if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine("B^-1 recalculated from B");
                _B_inv.copy_from(_B);
                bool singular_B = !sparse_matrix.invert(ref _B_inv, ref _E);
                if (singular_B)
                    return false;
            }
            else
            {
                double multiplier;

                _B_inv.divide_row(leaving_var, _dividers[leaving_var][0]);
                for (int cur_row = 0; cur_row < leaving_var; ++cur_row)
                {
                    multiplier = _dividers[cur_row][0];
                    if (multiplier != 0.0)
                        _B_inv.subtract_row(cur_row, leaving_var, multiplier);
                }
                for (int cur_row = leaving_var + 1; cur_row < num_vars; ++cur_row)
                {
                    multiplier = _dividers[cur_row][0];
                    if (multiplier != 0.0)
                        _B_inv.subtract_row(cur_row, leaving_var, multiplier);
                }
            }
            _B_inv.purge_zeroes();
            return true;
        }

        private bool solve_phase(int item_count, int num_bvars, ref int iterations)
        {
            int    art_var_index   = num_bvars - 2;
            bool   use_Blands_rule = false;
            double increase        = 0.0;

            while (true)
            {
                sparse_matrix.multiply(ref _z  , _cB, _B_inv);
                sparse_matrix.multiply(ref _z_c,  _z, _N);
                sparse_matrix.subtract(_z_c, _cN);
                _z_c.zero_roundoff_errors();
                sparse_matrix.multiply(ref _result, _B_inv, _b);
                _result.zero_roundoff_errors();

                int      entering_var  = -1;
                double   min_neg_coeff = 0.0;
                double[] z_c_ref       = _z_c[0];
                if (use_Blands_rule)
                {
                    for (int cur_var = 0; cur_var < item_count; ++cur_var)
                    {
                        if (z_c_ref[cur_var] < 0.0 && (_vN[cur_var] < art_var_index || _vN[cur_var] >= num_bvars))
                        {
                            min_neg_coeff = z_c_ref[cur_var];
                            entering_var  = cur_var;
                            break;
                        }
                    }
                }
                else
                {
                    for (int cur_var = 0; cur_var < item_count; ++cur_var)
                    {
                        if (min_neg_coeff > z_c_ref[cur_var] && (_vN[cur_var] < art_var_index || _vN[cur_var] >= num_bvars))
                        {
                            min_neg_coeff = z_c_ref[cur_var];
                            entering_var  = cur_var;
                        }
                    }
                }
                if (entering_var < 0)
                {
                    if (engine_control_unit.CALIBRATION_DEBUG)
                    {
                        if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                        {
                            sparse_matrix.multiply(ref _E, _B, _B_inv);
                            _E.log("E");
                            _E.set_to_identity();
                            _cB.log("cB");
                            _cN.log("cN");
                            _B.log("B");
                            _N.log("N");
                            _B_inv.log("B^-1");
                            _z.log("cB * B^-1");
                            _z_c.log("cB * B^-1 * N - cN");
                            _result.log("B^-1 * b");
                        }
                        MyLog.Default.WriteLine("<< FINISHED >>");
                        sparse_matrix.multiply(ref _total, _cB, _result);
                        MyLog.Default.WriteLine(string.Format("Iteration = {0}; total = {1}", iterations, _total[0][0]));
                    }
                    return true;
                }

                _entering_column.column_vector_from(_N, entering_var);
                sparse_matrix.multiply(ref _dividers, _B_inv, _entering_column);
                _dividers.zero_roundoff_errors();
                double min_ratio   = double.MaxValue, cur_ratio;
                int    leaving_var = -1;
                for (int cur_var = 0; cur_var < num_bvars; ++cur_var)
                {
                    if (_dividers[cur_var][0] > 0.0 && _result[cur_var][0] >= 0.0)
                    {
                        cur_ratio = _result[cur_var][0] / _dividers[cur_var][0];
                        if (min_ratio > cur_ratio)
                        {
                            min_ratio   = cur_ratio;
                            leaving_var = cur_var;
                        }
                    }
                }
                if (leaving_var < 0)
                {
                    MyLog.Default.WriteLine("<< ABORTED >>");
                    sparse_matrix.multiply(ref _total, _cB, _result);
                    log_var("Entering", _vN[entering_var], num_bvars);
                    MyLog.Default.WriteLine(string.Format("Iteration = {0}; total = {1}", iterations, _total[0][0]));
                    return false;
                }

                increase -= min_neg_coeff * min_ratio;
                if (engine_control_unit.CALIBRATION_DEBUG)
                {
                    if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                    {
                        sparse_matrix.multiply(ref _E, _B, _B_inv);
                        _E.log("E");
                        _E.set_to_identity();
                        _cB.log("cB");
                        _cN.log("cN");
                        _B.log("B");
                        _N.log("N");
                        _B_inv.log("B^-1");
                        _z.log("cB * B^-1");
                        _z_c.log("cB * B^-1 * N - cN");
                        _result.log("B^-1 * b");
                        _dividers.log("B^-1 * N[e]");
                    }
                    log_var("Entering", _vN[entering_var], num_bvars);
                    log_var( "Leaving", _vB[ leaving_var], num_bvars);
                    sparse_matrix.multiply(ref _total, _cB, _result);
                    MyLog.Default.WriteLine(string.Format("Iteration = {2}; increase = {0}; Bland's rule = {1}; total = {3}", -min_neg_coeff * min_ratio, use_Blands_rule, iterations, _total[0][0] - min_neg_coeff * min_ratio));
                }
                if ((iterations & 3) == 3)
                {
                    use_Blands_rule = increase < EPSILON;
                    increase        = 0.0;
                }
                bool singular_B = !perform_pivot(num_bvars, entering_var, leaving_var, iterations++);
                if (singular_B)
                {
                    MyLog.Default.WriteLine("<< SINGULAR >>");
                    _cB.log("cB");
                    _cN.log("cN");
                    _B.log("B");
                    _N.log("N");
                    return false;
                }

            }
        }

        private bool solve(int item_count)
        {
            int  num_vars = item_count + 2;
            bool singular_B;

            fill_matrices(item_count);

            sparse_row_ref cN_ref = _cN[0], cB_ref = _cB[0];
            cB_ref[0] = cB_ref[1] = -1.0;
            _B_inv.copy_from(_B);
            singular_B = !sparse_matrix.invert(ref _B_inv, ref _E);
            if (singular_B)
                return false;
            _B_inv.purge_zeroes();
            sparse_matrix.multiply(ref _result, _B_inv, _b);
            _result.zero_roundoff_errors();
            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
            {
                sparse_matrix.multiply(ref _E, _B, _B_inv);
                _E.log("E");
                _E.set_to_identity();
                _cB.log("cB");
                _cN.log("cN");
                _B.log("B");
                _N.log("N");
                _B_inv.log("B^-1");
                _result.log("B^-1 * b");
                sparse_matrix.multiply(ref _total, _cB, _result);
                _total.log("Total");
                MyLog.Default.WriteLine(string.Format("Total = {0}", _total[0][0]));
            }
            bool reinvert_B = false;
            for (int art_var = 0; art_var <= 1; ++art_var)
            {
                if (_result[art_var][0] < 0.0)
                {
                    sparse_row_ref B_row_ref = _B[art_var];

                    for (int cur_column = 2; cur_column < num_vars; ++cur_column)
                        B_row_ref[cur_column] = -B_row_ref[cur_column];
                    reinvert_B = true;
                }
            }
            if (reinvert_B)
            {
                _B_inv.copy_from(_B);
                singular_B = !sparse_matrix.invert(ref _B_inv, ref _E);
                if (singular_B)
                    return false;
                _B_inv.purge_zeroes();
            }
            int  num_iterations = 1;
            bool initial_BFS_present = solve_phase(item_count, num_vars, ref num_iterations);
            if (!initial_BFS_present)
                return false;
            sparse_matrix.multiply(ref _total, _cB, _result);
            if (_total[0][0] < -EPSILON)
                return false;
            initial_BFS_present = remove_artificial_variables(ref item_count, num_vars, ref num_iterations);
            if (!initial_BFS_present)
                return false;

            for (int cur_var = 0; cur_var < item_count; ++cur_var)
                cN_ref[cur_var] = (_vN[cur_var] >= num_vars) ? 0.0 : 1.0;
            for (int cur_var = 0; cur_var <   num_vars; ++cur_var)
                cB_ref[cur_var] = (_vB[cur_var] >= num_vars) ? 0.0 : 1.0;
            return solve_phase(item_count, num_vars, ref num_iterations);
        }

        private bool remove_artificial_variables(ref int item_count, int num_vars, ref int iterations)
        {
            for (int leaving_var = 0; leaving_var <= 1; ++leaving_var)
            {
                if (_vB[leaving_var] < item_count || _vB[leaving_var] >= num_vars)
                {
                    _art_var_pos[leaving_var] = -1;
                    for (int cur_var = 0; cur_var < item_count; ++cur_var)
                    {
                        if (_vN[cur_var] == item_count + leaving_var)
                        {
                            _art_var_pos[leaving_var] = cur_var;
                            break;
                        }
                    }
                    continue;
                }

                sparse_matrix.multiply(ref _coeffs, _B_inv, _N);
                _coeffs.zero_roundoff_errors();
                int entering_var = -1;
                double[] coeffs_row_ref = _coeffs[leaving_var];
                for (int cur_var = 0; cur_var < item_count; ++cur_var)
                {
                    if (coeffs_row_ref[cur_var] > 0.0 && (_vN[cur_var] < item_count || _vN[cur_var] >= num_vars))
                    {
                        entering_var = cur_var;
                        break;
                    }
                }
                if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                {
                    sparse_matrix.multiply(ref _E, _B, _B_inv);
                    _E.log("E");
                    _E.set_to_identity();
                    _cB.log("cB");
                    _cN.log("cN");
                    _B.log("B");
                    _N.log("N");
                    _B_inv.log("B^-1");
                    _coeffs.log("B^-1 * N");
                    log_var("Entering", _vN[entering_var], item_count);
                    log_var("Leaving", _vB[leaving_var], item_count);
                }
                if (entering_var < 0)
                    return false;

                _dividers.column_vector_from(_coeffs, entering_var);
                perform_pivot(num_vars, entering_var, leaving_var, iterations++);
                _art_var_pos[leaving_var] = entering_var;
            }

            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
            {
                sparse_matrix.multiply(ref _E, _B, _B_inv);
                _E.log("E");
                _E.set_to_identity();
                _cB.log("cB");
                _cN.log("cN");
                _B.log("B");
                _N.log("N");
                _B_inv.log("B^-1");
                MyLog.Default.WriteLine(string.Format("A1: #{0}, A2: #{1}", _art_var_pos[0] + 1, _art_var_pos[1] + 1));
            }
            int next_available_column = item_count - 2;
            for (int art_var = 0; art_var <= 1; ++art_var)
            {
                if (_art_var_pos[art_var] < 0 || _art_var_pos[art_var] >= item_count - 2)
                    continue;
                while (_vN[next_available_column] >= item_count && _vN[next_available_column] < num_vars)
                    ++next_available_column;
                if (engine_control_unit.CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine(string.Format("#{0} <--> #{1}", _art_var_pos[art_var] + 1, next_available_column + 1));
                sparse_matrix.exchange_columns(_cN, _art_var_pos[art_var], _cN, next_available_column);
                sparse_matrix.exchange_columns (_N, _art_var_pos[art_var],  _N, next_available_column);
                _vN[_art_var_pos[art_var]] = _vN[next_available_column];
                _vN[next_available_column] = art_var + item_count;
            }
            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
            {
                _cN.log("cN");
                _N.log("N");
            }
            _cN.shrink_width(2);
            _N.shrink_width(2);
            _z_c.shrink_width(2);
            item_count -= 2;
            return true;
        }

        public revised_simplex_solver()
        {
            items = new List<solver_entry>();
        }

        public bool calculate_solution(int item_count)
        {
            for (int cur_item = 0; cur_item < item_count; ++cur_item)
                items[cur_item].result = 0.0f;
            if (item_count < 3)
                return false;
            if (!solve(item_count))
                return false;

            int num_vars = item_count + 2;
            for (int cur_var = 0; cur_var < num_vars; ++cur_var)
            {
                if (_vB[cur_var] < item_count)
                    items[_vB[cur_var]].result = (float) _result[cur_var][0];
            }

            return is_solution_good(item_count);
        }
    }

    sealed class simplex_solver
    {
        double[][] _tableau;
        int _num_rows, _num_columns;

        private void log_tableau(double[][] tableau, int num_rows, int num_columns, int pivot_row, int pivot_column, int[] side_index)
        {
            StringBuilder line = new StringBuilder();

            MyLog.Default.WriteLine("============================");
            for (int cur_row = 0; cur_row < num_rows; ++cur_row)
            {
                int cur_side = 0;

                line.Clear();
                for (int cur_column = 0; cur_column < num_columns; ++cur_column)
                {
                    if (cur_side < 7 && cur_column >= side_index[cur_side])
                    {
                        line.Append("||");
                        ++cur_side;
                    }
                    line.Append(string.Format("{0,8:G2}{1}", tableau[cur_row][cur_column], (cur_row == pivot_row || cur_column == pivot_column) ? '*' : ' '));
                }
                line.AppendLine("|");
                MyLog.Default.WriteLine(line.ToString());
            }
        }


        public bool solve(double[][] init_tableau, int num_rows, int num_columns, int[] side_index)
        {
            //log_tableau(init_tableau, num_rows, num_columns, -1, -1, side_index);

            _num_rows    = num_rows;
            _num_columns = num_columns;
            _tableau = new double[num_rows][];
            for (int cur_row = 0; cur_row < num_rows; ++cur_row)
            {
                _tableau[cur_row] = new double[num_columns];
                Array.Copy(init_tableau[cur_row], _tableau[cur_row], num_columns);
            }

            while (true)
            {
                double min_negative = 0.0;
                int    pivot_column = -1;
                for (int cur_coulumn = 0; cur_coulumn < _num_columns - 1; ++cur_coulumn)
                {
                    if (_tableau[_num_rows - 1][cur_coulumn] < min_negative)
                    {
                        min_negative = _tableau[_num_rows - 1][cur_coulumn];
                        pivot_column = cur_coulumn;
                    }
                }
                if (pivot_column < 0)
                {
                    //log_tableau(_tableau, num_rows, num_columns, -1, -1, side_index);
                    return true;
                }

                double min_ratio = double.MaxValue;
                int    pivot_row = -1;
                for (int cur_row = 0; cur_row < _num_rows - 1; ++cur_row)
                {
                    if (_tableau[cur_row][pivot_column] > 0.0 && _tableau[cur_row][_num_columns - 1] >= 0.0)
                    {
                        double cur_ratio = _tableau[cur_row][_num_columns - 1] / _tableau[cur_row][pivot_column];
                        if (cur_ratio < min_ratio)
                        {
                            min_ratio = cur_ratio;
                            pivot_row = cur_row;
                        }
                    }
                }
                if (pivot_row < 0)
                {
                    //log_tableau(_tableau, num_rows, num_columns, -1, pivot_column, side_index);
                    return false;
                }

                //log_tableau(_tableau, num_rows, num_columns, pivot_row, pivot_column, side_index);
                double divider = _tableau[pivot_row][pivot_column];
                for (int cur_column = 0; cur_column < _num_columns; ++cur_column)
                    _tableau[pivot_row][cur_column] /= divider;
                _tableau[pivot_row][pivot_column] = 1.0;
                for (int cur_row = 0; cur_row < _num_rows; ++cur_row)
                {
                    if (cur_row == pivot_row)
                        continue;

                    double multiplier = _tableau[cur_row][pivot_column];
                    for (int cur_column = 0; cur_column < _num_columns; ++cur_column)
                    {
                        _tableau[cur_row][cur_column] -= _tableau[pivot_row][cur_column] * multiplier;
                        if (Math.Abs(_tableau[cur_row][cur_column]) < revised_simplex_solver.EPSILON)
                            _tableau[cur_row][cur_column] = 0.0;
                    }
                    _tableau[cur_row][pivot_column] = 0.0;
                }
            }
        }

        public float extract_item_value(int item)
        {
            float result = 0.0f;
            int   cur_row = 0;

            while (cur_row < _num_rows - 1 && Math.Abs(_tableau[cur_row][item]) < revised_simplex_solver.EPSILON)
                ++cur_row;
            if (cur_row >= _num_rows - 1)
                return 0.0f;
            result = (float) (_tableau[cur_row][_num_columns - 1] / _tableau[cur_row][item]);
            do
            {
                ++cur_row;
            }
            while (cur_row < _num_rows - 1 && Math.Abs(_tableau[cur_row][item]) < revised_simplex_solver.EPSILON);
            //MyLog.Default.WriteLine(string.Format("[{0}] = {1}", item, (cur_row >= _num_rows - 1) ? result : 0.0f));
            return (cur_row >= _num_rows - 1) ? result : 0.0f;
        }
    }

    sealed class solver_entry6
    {
        public float   x, y, z, max_value;
        public float[] results;
    };

    sealed class revised_simplex_solver6
    {
        const int NUM_ART_VARS = 3, NUM_EXTRA_VARS = 1, COT_CONSTRAINTS = 6;

        public const double EPSILON = revised_simplex_solver.EPSILON, BIG_M = 100.0;

        private int                 _last_item_count = 0;
        private int[]               _vB = null, _vN = null;
        private Dictionary<int, int> _art_var_pos = new Dictionary<int, int>();
        private sparse_matrix       _E, _cB, _cN, _B0, _B, _N0, _N, _B_inv;
        private dense_matrix        _b0, _b, _z, _z_c;
        private sparse_matrix       _entering_column;
        private dense_matrix        _result, _dividers, _total, _coeffs;

        static private string var_name(int var, int item_count)
        {
            int n1 = 0, n2 = 0, n3 = 0;

            if (var < item_count)
                n1 = var + 1;
            else if (var < item_count + NUM_ART_VARS)
                n2 = var - item_count + 1;
            else
                n3 = var - item_count - NUM_ART_VARS + 1;
            return string.Format("{0}:{1}:{2}", n1, n2, n3);
        }

        static private void log_var(string prefix, int var, int item_count)
        {
            MyLog.Default.WriteLine(string.Format("{0} #{1}", prefix, var_name(var, item_count)));
        }

        private void fill_matrices(double mass, List<solver_entry6> items, int[] side_index, out int num_bvars, out int num_nvars, float dead_zone_radius)
        {
            int item_count = side_index[6];
            num_bvars = item_count + NUM_ART_VARS + COT_CONSTRAINTS;
            num_nvars = item_count + NUM_EXTRA_VARS;
            if (item_count != _last_item_count)
            {
                _vB = new int[num_bvars];
                _vN = new int[num_nvars];
                _last_item_count = item_count;
            }

            _E.set_to_identity(num_bvars);
            _B0.set_to_identity(num_bvars);
            _B.clear(num_bvars);
            _B_inv.clear(num_bvars);
            _N0.clear(num_bvars, num_nvars);
            _N.clear(num_bvars, num_nvars);
            _b0.clear(num_bvars, 1);
            _b.clear(num_bvars, 1);
            //_B_inv.set_to_identity(num_vars);
            _z.clear  (1, num_bvars);
            _z_c.clear(1, num_nvars);

            _result.clear         (num_bvars, 1);
            _entering_column.clear(num_bvars, 1);
            _dividers.clear       (num_bvars, 1);

            int cur_index;
            for (int cur_item = 0; cur_item < item_count; ++cur_item)
            {
                _N0[NUM_ART_VARS    ][cur_item] =  items[cur_item].x;
                _N0[NUM_ART_VARS + 1][cur_item] = -items[cur_item].x;
                _N0[NUM_ART_VARS + 2][cur_item] =  items[cur_item].y;
                _N0[NUM_ART_VARS + 3][cur_item] = -items[cur_item].y;
                _N0[NUM_ART_VARS + 4][cur_item] =  items[cur_item].z;
                _N0[NUM_ART_VARS + 5][cur_item] = -items[cur_item].z;

                cur_index = cur_item + NUM_ART_VARS + COT_CONSTRAINTS;
                _N0[cur_index][cur_item] = 1.0;
                _b0[cur_index][0] = items[cur_item].max_value;
            }
            _N0[0][num_nvars - 1] = -1.0;
            for (cur_index = NUM_ART_VARS; cur_index < NUM_ART_VARS + COT_CONSTRAINTS; ++cur_index)
                _b0[cur_index][0] = mass * dead_zone_radius / 3.0;
        }

        private void swap_vars(int entering_var, int leaving_var)
        {
            int temp          = _vN[entering_var];
            _vN[entering_var] = _vB[leaving_var];
            _vB[leaving_var]  = temp;
            sparse_matrix.exchange_columns(_cN, entering_var, _cB, leaving_var);
            sparse_matrix.exchange_columns( _N, entering_var,  _B, leaving_var);
        }

        private bool perform_pivot(int num_bvars, int entering_var, int leaving_var, int iterations)
        {
            swap_vars(entering_var, leaving_var);
            if ((iterations & 0xF) == 0)
            {
                if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine("B^-1 recalculated from B");
                _B_inv.copy_from(_B);
                bool singular_B = !sparse_matrix.invert(ref _B_inv, ref _E);
                if (singular_B)
                    return false;
            }
            else
            {
                double multiplier;

                _B_inv.divide_row(leaving_var, _dividers[leaving_var][0]);
                for (int cur_row = 0; cur_row < leaving_var; ++cur_row)
                {
                    multiplier = _dividers[cur_row][0];
                    if (multiplier != 0.0)
                        _B_inv.subtract_row(cur_row, leaving_var, multiplier);
                }
                for (int cur_row = leaving_var + 1; cur_row < num_bvars; ++cur_row)
                {
                    multiplier = _dividers[cur_row][0];
                    if (multiplier != 0.0)
                        _B_inv.subtract_row(cur_row, leaving_var, multiplier);
                }
            }
            _B_inv.purge_zeroes();
            return true;
        }

        private bool solve_phase(int item_count, int num_bvars, int num_nvars, ref int iterations)
        {
            int    art_var_index   = num_bvars - NUM_ART_VARS, first_entering_var = -1;
            bool   use_Blands_rule = false;
            double increase        = 0.0;

            sparse_matrix.multiply(ref _result, _B_inv, _b);
            _result.zero_roundoff_errors();
            while (true)
            {
                sparse_matrix.multiply(ref _z  , _cB, _B_inv);
                sparse_matrix.multiply(ref _z_c,  _z, _N);
                sparse_matrix.subtract(_z_c, _cN);
                _z_c.zero_roundoff_errors();

                int      entering_var  = -1;
                double   min_neg_coeff = 0.0;
                double[] z_c_ref       = _z_c[0];
                if (use_Blands_rule)
                {
                    int min_index = int.MaxValue;

                    for (int cur_var = 0; cur_var < num_nvars; ++cur_var)
                    {
                        if (z_c_ref[cur_var] < 0.0 && _vN[cur_var] > first_entering_var && _vN[cur_var] < min_index)
                        {
                            min_index     = _vN[cur_var];
                            min_neg_coeff = z_c_ref[cur_var];
                            entering_var  = cur_var;
                        }
                    }
                }
                else
                {
                    for (int cur_var = 0; cur_var < num_nvars; ++cur_var)
                    {
                        if (min_neg_coeff > z_c_ref[cur_var])
                        {
                            min_neg_coeff = z_c_ref[cur_var];
                            entering_var  = cur_var;
                        }
                    }
                }
                if (entering_var < 0)
                {
                    if (engine_control_unit.CALIBRATION_DEBUG)
                    {
                        if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                        {
                            sparse_matrix.multiply(ref _E, _B, _B_inv);
                            _E.log("E");
                            _E.set_to_identity();
                            _cB.log("cB");
                            _cN.log("cN");
                            _B.log("B");
                            _N.log("N");
                            _B_inv.log("B^-1");
                            _z.log("cB * B^-1");
                            _z_c.log("cB * B^-1 * N - cN");
                            _result.log("B^-1 * b");
                        }
                        MyLog.Default.WriteLine("<< FINISHED >>");
                        sparse_matrix.multiply(ref _total, _cB, _result);
                        MyLog.Default.WriteLine(string.Format("Iteration = {0}; total = {1}", iterations, _total[0][0]));
                    }
                    return true;
                }

                _entering_column.column_vector_from(_N, entering_var);
                sparse_matrix.multiply(ref _dividers, _B_inv, _entering_column);
                _dividers.zero_roundoff_errors();
                double min_ratio   = double.MaxValue, cur_ratio;
                int    leaving_var = -1;
                for (int cur_var = 0; cur_var < num_bvars; ++cur_var)
                {
                    if (_dividers[cur_var][0] > 0.0 && _result[cur_var][0] >= 0.0)
                    {
                        cur_ratio = _result[cur_var][0] / _dividers[cur_var][0];
                        if (min_ratio > cur_ratio)
                        {
                            min_ratio   = cur_ratio;
                            leaving_var = cur_var;
                        }
                    }
                }
                if (leaving_var < 0)
                {
                    MyLog.Default.WriteLine("<< ABORTED >>");
                    sparse_matrix.multiply(ref _total, _cB, _result);
                    log_var("Entering", _vN[entering_var], item_count);
                    MyLog.Default.WriteLine(string.Format("Iteration = {0}; total = {1}", iterations, _total[0][0]));
                    return false;
                }

                if (engine_control_unit.CALIBRATION_DEBUG)
                {
                    if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                    {
                        sparse_matrix.multiply(ref _E, _B, _B_inv);
                        _E.log("E");
                        _E.set_to_identity();
                        _cB.log("cB");
                        _cN.log("cN");
                        _B.log("B");
                        _N.log("N");
                        _B_inv.log("B^-1");
                        _z.log("cB * B^-1");
                        _z_c.log("cB * B^-1 * N - cN");
                        _result.log("B^-1 * b");
                        _dividers.log("B^-1 * N[e]");
                    }
                    log_var("Entering", _vN[entering_var], item_count);
                    log_var( "Leaving", _vB[ leaving_var], item_count);
                    sparse_matrix.multiply(ref _total, _cB, _result);
                    MyLog.Default.WriteLine(string.Format("Iteration = {2}; increase = {0}; Bland's rule = {1}; total = {3}", -min_neg_coeff * min_ratio, use_Blands_rule, iterations, _total[0][0] - min_neg_coeff * min_ratio));
                }
                bool singular_B = !perform_pivot(num_bvars, entering_var, leaving_var, iterations++);
                if (singular_B)
                {
                    MyLog.Default.WriteLine("<< SINGULAR >>");
                    _cB.log("cB");
                    _cN.log("cN");
                    _B.log("B");
                    _N.log("N");
                    return false;
                }

                sparse_matrix.multiply(ref _result, _B_inv, _b);
                _result.zero_roundoff_errors();
                int error_row;
                for (error_row = 0; error_row < num_bvars; ++error_row)
                {
                    if (_result[error_row][0] < 0.0)
                        break;
                }
                if (error_row >= num_bvars)
                    first_entering_var = -1;
                else
                {
                    --iterations;
                    singular_B = !perform_pivot(num_bvars, entering_var, leaving_var, 0);
                    if (singular_B)
                    {
                        MyLog.Default.WriteLine("<< SINGULAR >>");
                        _cB.log("cB");
                        _cN.log("cN");
                        _B.log("B");
                        _N.log("N");
                        return false;
                    }
                    if (use_Blands_rule)
                        first_entering_var = _vN[entering_var];
                    else
                        use_Blands_rule = true;
                    MyLog.Default.WriteLine(string.Format("#{0} < 0",  var_name(_vN[entering_var], item_count)));
                    sparse_matrix.multiply(ref _result, _B_inv, _b);
                    _result.zero_roundoff_errors();
                }
                increase -= min_neg_coeff * min_ratio;
                if ((iterations & 3) == 3)
                {
                    use_Blands_rule = error_row < num_bvars || increase < EPSILON;
                    increase        = 0.0;
                }
            }
        }

        private bool solve(List<solver_entry6> items, int[] side_index, int cur_side, int num_bvars, int num_nvars)
        {
            int  item_count = side_index[6];
            bool singular_B;

            if (engine_control_unit.CALIBRATION_DEBUG)
                MyLog.Default.WriteLine("Side = " + cur_side.ToString());
            for (int cur_Fvar = 0; cur_Fvar < item_count; ++cur_Fvar)
                _vN[cur_Fvar] = cur_Fvar;
            _vN[item_count] = item_count + num_bvars;
            for (int cur_asvar = 0; cur_asvar < num_bvars; ++cur_asvar)
                _vB[cur_asvar] = cur_asvar + item_count;
            _B.copy_from(_B0);
            _N.copy_from(_N0);
            _b.copy_from(_b0);
            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                _b.log("b");
            double F_coeff = 1.0;
            int    side    = cur_side;
            do
            {
                for (int cur_column = side_index[side]; cur_column < side_index[side + 1]; ++cur_column)
                    _N[0][cur_column] = F_coeff;
                side = (side + 1) % 6;
                for (int perpendicular_side = 1; perpendicular_side <= 2; ++perpendicular_side)
                {
                    for (int cur_column = side_index[side]; cur_column < side_index[side + 1]; ++cur_column)
                        _N[perpendicular_side][cur_column] = F_coeff;
                    side = (side + 1) % 6;
                }
                F_coeff = -F_coeff;
            }
            while (side != cur_side);

            _cB.clear(1, num_bvars);
            _cN.clear(1, num_nvars);
            sparse_row_ref cN_ref = _cN[0], cB_ref = _cB[0];
            int opposite_side = (cur_side + 3) % 6;
            for (int cur_var = side_index[cur_side]; cur_var < side_index[cur_side + 1]; ++cur_var)
                cN_ref[cur_var] = 1.0;
            for (int cur_var = side_index[opposite_side]; cur_var < side_index[opposite_side + 1]; ++cur_var)
                cN_ref[cur_var] = -1.0;
            for (int cur_var = 0; cur_var < NUM_ART_VARS; ++cur_var)
                cB_ref[cur_var] = -BIG_M;

            int num_iterations = 1;
            _B_inv.copy_from(_B);
            bool initial_BFS_present = remove_artificial_variables(item_count, num_bvars, ref num_nvars, ref num_iterations);
            if (!initial_BFS_present)
                return false;

            if (!solve_phase(item_count, num_bvars, num_nvars, ref num_iterations))
                return false;

            sparse_matrix.multiply(ref _total, _cB, _result);
            _b[0][0] = _total[0][0] * 0.99999;
            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                _b.log("b");
            for (int cur_var = 0; cur_var < num_nvars; ++cur_var)
            {
                if (_vN[cur_var] < side_index[6])
                    cN_ref[cur_var] = -1.0;
            }
            for (int cur_var = 0; cur_var < num_bvars; ++cur_var)
            {
                if (_vB[cur_var] < side_index[6])
                    cB_ref[cur_var] = -1.0;
            }
            if (!solve_phase(item_count, num_bvars, num_nvars, ref num_iterations))
                return false;

            _E.set_to_identity();
            _B_inv.copy_from(_B);
            singular_B = !sparse_matrix.invert(ref _B_inv, ref _E);
            if (singular_B)
                return false;
            _B_inv.purge_zeroes();
            sparse_matrix.multiply(ref _E, _B, _B_inv);
            _E.purge_zeroes();
            if (engine_control_unit.CALIBRATION_DEBUG)
            {
                double min_B = double.MaxValue, max_B_inv = 0.0;
                for (int x = 0; x < num_bvars; ++x)
                {
                    for (int y = 0; y < num_bvars; ++y)
                    {
                        if (x == y && Math.Abs(_E[x][y] - 1.0) > EPSILON || x != y && Math.Abs(_E[x][y]) > EPSILON)
                            MyLog.Default.WriteLine("Faulty inversion");
                        if (_B[x][y] != 0.0 && min_B > Math.Abs(_B[x][y]))
                            min_B = Math.Abs(_B[x][y]);
                        if (max_B_inv < Math.Abs(_B_inv[x][y]))
                            max_B_inv = Math.Abs(_B_inv[x][y]);
                    }
                }
                MyLog.Default.WriteLine(string.Format("Bmin = {0}, B^-1max = {1}", min_B, max_B_inv));
            }
            sparse_matrix.slow_multiply(ref _result, _B_inv, _b);
            _result.zero_roundoff_errors();
            if (engine_control_unit.CALIBRATION_DEBUG)
            {
                for (int cur_row = 0; cur_row < num_bvars; ++cur_row)
                    MyLog.Default.WriteLine(string.Format("{0} = {1}", var_name(_vB[cur_row], item_count), _result[cur_row][0]));
            }
            return true;
        }

        private bool remove_artificial_variables(int item_count, int num_bvars, ref int num_nvars, ref int iterations)
        {
            _art_var_pos.Clear();
            for (int cur_nvar = 0; cur_nvar < num_nvars; ++cur_nvar)
            {
                int art_var = _vN[cur_nvar];
                if (art_var >= item_count && art_var < num_bvars)
                    _art_var_pos[art_var] = cur_nvar;
            }

            for (int leaving_var = 0; leaving_var < num_bvars; ++leaving_var)
            {
                int art_var = _vB[leaving_var];
                if (art_var < item_count || art_var >= item_count + NUM_ART_VARS)
                    continue;

                sparse_matrix.multiply(ref _coeffs, _B_inv, _N);
                _coeffs.zero_roundoff_errors();
                int entering_var = -1;
                double[] coeffs_row_ref = _coeffs[art_var - item_count];
                for (int cur_nvar = 0; cur_nvar < num_nvars; ++cur_nvar)
                {
                    if (coeffs_row_ref[cur_nvar] > 0.0 && !_art_var_pos.ContainsKey(_vN[cur_nvar]))
                    {
                        entering_var = cur_nvar;
                        break;
                    }
                }
                if (engine_control_unit.FULL_CALIBRATION_DEBUG)
                {
                    sparse_matrix.multiply(ref _E, _B, _B_inv);
                    _E.log("E");
                    _E.set_to_identity();
                    _cB.log("cB");
                    _cN.log("cN");
                    _B.log("B");
                    _N.log("N");
                    _B_inv.log("B^-1");
                    _coeffs.log("B^-1 * N");
                }
                if (engine_control_unit.CALIBRATION_DEBUG && entering_var >= 0)
                {
                    log_var("Entering", _vN[entering_var], item_count);
                    log_var( "Leaving", _vB[ leaving_var], item_count);
                    MyLog.Default.WriteLine(string.Format("Iteration = {0}", iterations));
                }
                if (entering_var >= 0)
                {
                    _dividers.column_vector_from(_coeffs, entering_var);
                    perform_pivot(num_bvars, entering_var, leaving_var, iterations++);
                    _art_var_pos[art_var] = entering_var;
                }
                else
                {
                    ++iterations;
                    for (int cur_nvar = 0; cur_nvar < num_nvars; ++cur_nvar)
                    {
                        if (coeffs_row_ref[cur_nvar] < 0.0)
                            _art_var_pos[_vN[cur_nvar]] = cur_nvar;
                    }
                }
            }

            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
            {
                sparse_matrix.multiply(ref _E, _B, _B_inv);
                _E.log("E");
                _E.set_to_identity();
                _cB.log("cB");
                _cN.log("cN");
                _B.log("B");
                _N.log("N");
                _B_inv.log("B^-1");

                StringBuilder art_var_line = new StringBuilder();
                foreach (var cur_art_var in _art_var_pos)
                {
                    if (art_var_line.Length > 0)
                        art_var_line.Append(", ");
                    art_var_line.Append(string.Format("A{0} -> {1}", var_name(cur_art_var.Key, item_count), cur_art_var.Value));
                }
                MyLog.Default.WriteLine(art_var_line.ToString());
            }
            int num_art_vars = _art_var_pos.Count, first_available_column = num_nvars - num_art_vars, next_available_column = first_available_column;
            foreach (var cur_art_var in _art_var_pos)
            {
                int cur_art_var_pos = cur_art_var.Value;
                if (cur_art_var_pos >= first_available_column)
                    continue;
                while (_art_var_pos.ContainsKey(_vN[next_available_column]))
                    ++next_available_column;
                if (engine_control_unit.CALIBRATION_DEBUG)
                    MyLog.Default.WriteLine(string.Format("#{0} <--> #{1}", cur_art_var_pos, next_available_column));
                sparse_matrix.exchange_columns(_cN, cur_art_var_pos, _cN, next_available_column);
                sparse_matrix.exchange_columns (_N, cur_art_var_pos,  _N, next_available_column);
                _vN[      cur_art_var_pos] = _vN[next_available_column];
                _vN[next_available_column] = cur_art_var.Key;
            }
            if (engine_control_unit.FULL_CALIBRATION_DEBUG)
            {
                _cN.log("cN");
                _N.log("N");
            }
            _cN.shrink_width(num_art_vars);
            _N.shrink_width(num_art_vars);
            _z_c.shrink_width(num_art_vars);
            num_nvars -= num_art_vars;
            return num_nvars > 0;
        }

        public bool calculate_solution(float mass, List<solver_entry6> items, int[] side_index, float dead_zone_radius)
        {
            for (int cur_item = 0; cur_item < side_index[6]; ++cur_item)
                items[cur_item].results = new float[6];
            if (side_index[6] < NUM_ART_VARS)
                return false;

            int bvars, nvars;
            fill_matrices(mass, items, side_index, out bvars, out nvars, dead_zone_radius);

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (solve(items, side_index, dir_index, bvars, nvars))
                {
                    for (int cur_var = 0; cur_var < bvars; ++cur_var)
                    {
                        if (_vB[cur_var] < side_index[6])
                        {
                            if (engine_control_unit.CALIBRATION_DEBUG && _result[cur_var][0] > items[_vB[cur_var]].max_value)
                                MyLog.Default.WriteLine(string.Format("({3})[{0}] = {1}/{2}", _vB[cur_var], _result[cur_var][0], items[_vB[cur_var]].max_value, cur_var));
                            items[_vB[cur_var]].results[dir_index] = (float) _result[cur_var][0];
                        }
                    }
                }
            }

            return true;
        }
    }
}
