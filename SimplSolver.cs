using System.Collections.Generic;

using VRage.Utils;

namespace orbiter_SE
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

            int[] vB = _vB, vN = _vN;
            List<solver_entry> items = this.items;

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
                vB    [cur_index] = cur_item;
                vN    [cur_item ] = cur_item + num_vars;
                B0_ref[cur_index] = items[cur_item].x;
                B1_ref[cur_index] = items[cur_item].y;
                _N[cur_index][cur_item] = 1.0;
                _b[cur_index][       0] = items[cur_item].max_value;
            }
            vB[0] = item_count;
            vB[1] = item_count + 1;
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

        private bool solve_phase(int item_count, int num_vars, ref int iterations)
        {
            int    art_var_index   = num_vars - 2;
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
                        if (z_c_ref[cur_var] < 0.0 && (_vN[cur_var] < art_var_index || _vN[cur_var] >= num_vars))
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
                        if (min_neg_coeff > z_c_ref[cur_var] && (_vN[cur_var] < art_var_index || _vN[cur_var] >= num_vars))
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
                for (int cur_var = 0; cur_var < num_vars; ++cur_var)
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
                    log_var("Entering", _vN[entering_var], num_vars);
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
                    log_var("Entering", _vN[entering_var], num_vars);
                    log_var( "Leaving", _vB[ leaving_var], num_vars);
                    sparse_matrix.multiply(ref _total, _cB, _result);
                    MyLog.Default.WriteLine(string.Format("Iteration = {2}; increase = {0}; Bland's rule = {1}; total = {3}", -min_neg_coeff * min_ratio, use_Blands_rule, iterations, _total[0][0] - min_neg_coeff * min_ratio));
                }
                if ((iterations & 3) == 3)
                {
                    use_Blands_rule = increase < EPSILON;
                    increase        = 0.0;
                }
                bool singular_B = !perform_pivot(num_vars, entering_var, leaving_var, iterations++);
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
}
