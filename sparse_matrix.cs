using System;
using System.Collections.Generic;
using System.Text;

using VRage.Utils;

namespace ttdtwm
{
    interface sparse_row_ref
    {
        double this[int column] { get; set; }
    }

    struct sparse_matrix
    {
        #region Row reference

        private sealed class sparse_row: sparse_row_ref
        {
            private int                     _width;
            private Dictionary<int, double> _row_ref;

            private sparse_row()
            {
                throw new InvalidOperationException("Sparse matrix row not initialised properly");
            }

            public double this[int column]
            {
                get
                {
                    if (column < 0 || column >= _width)
                        throw new ArgumentException("Matrix column out of range");

                    double result;

                    _row_ref.TryGetValue(column, out result);
                    return result;
                }
                set
                {
                    if (column < 0 || column >= _width)
                        throw new ArgumentException("Matrix column out of range");

                    if (value != 0.0)
                        _row_ref[column] = value;
                    else if (_row_ref.ContainsKey(column))
                        _row_ref.Remove(column);
                }
            }

            public sparse_row(Dictionary<int, double> row_ref, int columns)
            {
                _width   = columns;
                _row_ref = row_ref;
            }

            public void set_width(int columns)
            {
                for (int cur_column = columns; cur_column < _width; ++cur_column)
                {
                    if (_row_ref.ContainsKey(cur_column))
                        _row_ref.Remove(cur_column);
                }
                _width = columns;
            }
        }

        #endregion

        static private    int[] __index_array = null;
        static private double[] __neg_row     = null;

        private int                       _width, _initial_width, _height;
        private Dictionary<int, double>[] _contents;
        private sparse_row[]              _row_objects;

        public sparse_row_ref this[int row]
        {
            get
            {
                return _row_objects[row];
            }
        }

        #region Assigment operations

        public void clear(int new_rows = 0, int new_columns = 0)
        {
            if (new_rows <= 0)
            {
                if (_height <= 0)
                    throw new ArgumentException("Matrix size must be positive");
                new_rows    = _height;
                new_columns = _initial_width;
            }
            else if (new_columns <= 0)
                new_columns = new_rows;

            _width = _initial_width = new_columns;
            if (new_rows == _height)
            {
                for (int cur_row = 0; cur_row < _height; ++cur_row)
                {
                    _contents   [cur_row].Clear();
                    _row_objects[cur_row].set_width(_initial_width);
                }
            }
            else
            {
                _height      = new_rows;
                _contents    = new Dictionary<int, double>[_height];
                _row_objects = new              sparse_row[_height];
                for (int cur_row = 0; cur_row < _height; ++cur_row)
                {
                    _contents   [cur_row] = new Dictionary<int, double>();
                    _row_objects[cur_row] = new sparse_row(_contents[cur_row], _initial_width);
                }
            }

            if (__index_array == null || __index_array.Length < _initial_width)
                __index_array = new int[_initial_width];
        }

        public void set_to_identity(int size = 0)
        {
            clear(size);
            for (int cur_element = 0; cur_element < _height; ++cur_element)
                _contents[cur_element].Add(cur_element, 1.0);
        }

        public void copy_from(sparse_matrix b)
        {
            clear(b._height, b._width);
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                IDictionary<int, double> row_ref = _contents[cur_row], src_row_ref = b._contents[cur_row];

                foreach (var cur_element in src_row_ref)
                    row_ref.Add(cur_element);
            }
        }

        public void column_vector_from(sparse_matrix b, int column)
        {
            double                  value;
            bool                    non_zero;
            Dictionary<int, double> row_ref;

            clear(b._height, 1);
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                row_ref  =   _contents[cur_row];
                non_zero = b._contents[cur_row].TryGetValue(column, out value);
                if (non_zero)
                    row_ref[0] = value;
                else if (row_ref.ContainsKey(0))
                    row_ref.Remove(0);
            }
        }

        private void exchange_rows(int row1, int row2)
        {
            Dictionary<int, double> temp_row = _contents[row1];
            _contents[row1]                  = _contents[row2];
            _contents[row2]                  = temp_row;
            sparse_row temp_row_obj = _row_objects[row1];
            _row_objects[row1]      = _row_objects[row2];
            _row_objects[row2]      = temp_row_obj;
        }

        public static void exchange_columns(sparse_matrix a, int column_a, sparse_matrix b, int column_b)
        {
            if (a._height != b._height)
                throw new ArgumentException("Row number mismatch");

            double                  a_value, b_value;
            bool                    non_zero_a, non_zero_b;
            Dictionary<int, double> a_row_ref, b_row_ref;

            for (int cur_row = 0; cur_row < a._height; ++cur_row)
            {
                a_row_ref = a._contents[cur_row];
                b_row_ref = b._contents[cur_row];
                non_zero_a = a_row_ref.TryGetValue(column_a, out a_value);
                non_zero_b = b_row_ref.TryGetValue(column_b, out b_value);
                if (non_zero_a)
                {
                    if (non_zero_b)
                    {
                        a_row_ref[column_a] = b_value;
                        b_row_ref[column_b] = a_value;
                    }
                    else
                    {
                        b_row_ref.Add(column_b, a_value);
                        a_row_ref.Remove(column_a);
                    }
                }
                else if (non_zero_b)
                {
                    a_row_ref.Add(column_a, b_value);
                    b_row_ref.Remove(column_b);
                }
            }
        }

        public void shrink_width(int shrink)
        {
            if (shrink < 0 || _width <= shrink)
                throw new ArgumentException("Shrink amount negative or too big");

            _width -= shrink;
            for (int cur_row = 0; cur_row < _height; ++cur_row)
                _row_objects[cur_row].set_width(_width);
        }

        public void purge_zeroes()
        {
            Dictionary<int, double> cur_row_ref;

            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                cur_row_ref    = _contents[cur_row];
                int num_zeroes = 0;

                foreach (var cur_element in cur_row_ref)
                {
                    if (Math.Abs(cur_element.Value) < revised_simplex_solver.EPSILON)
                        __index_array[num_zeroes++] = cur_element.Key;
                }
                for (int cur_index = 0; cur_index < num_zeroes; ++cur_index)
                    cur_row_ref.Remove(__index_array[cur_index]);
            }
        }

        #endregion

        #region Arithmetic operations

        public static void subtract(dense_matrix minuend, sparse_matrix subtrahend)
        {
            if (minuend.columns != subtrahend._width || minuend.rows != subtrahend._height)
                throw new ArgumentException("Attempt to subtract matrix of different size");

            double[]                minuend_row_ref;
            Dictionary<int, double> subtrahend_row_ref;
            for (int cur_row = 0; cur_row < subtrahend._height; ++cur_row)
            {
                minuend_row_ref    =              minuend[cur_row];
                subtrahend_row_ref = subtrahend._contents[cur_row];
                foreach (var cur_element in subtrahend_row_ref)
                    minuend_row_ref[cur_element.Key] -= cur_element.Value;
            }
        }

        static public void multiply(ref sparse_matrix result, sparse_matrix left, sparse_matrix right)
        {
            if (left._width != right._height)
                throw new ArgumentException("Operand size mismatch");

            Dictionary<int, double> result_row_ref, left_row_ref, right_row_ref;
            double                  cur_element;
            int                     cur_column;

            result.clear(left._height, right._width);
            for (int cur_row = 0; cur_row < result._height; ++cur_row)
            {
                left_row_ref   =   left._contents[cur_row];
                result_row_ref = result._contents[cur_row];
                foreach (var cur_diagonal in left_row_ref)
                {
                    cur_element   = cur_diagonal.Value;
                    right_row_ref = right._contents[cur_diagonal.Key];
                    foreach (var cur_horizontal in right_row_ref)
                    {
                        cur_column = cur_horizontal.Key;
                        if (result_row_ref.ContainsKey(cur_column))
                            result_row_ref[cur_column] += cur_element * cur_horizontal.Value;
                        else
                            result_row_ref.Add(cur_column, cur_element * cur_horizontal.Value);
                    }
                }
            }
        }

        static public void multiply(ref dense_matrix result, sparse_matrix left, sparse_matrix right)
        {
            if (left._width != right._height)
                throw new ArgumentException("Operand size mismatch");

            Dictionary<int, double> left_row_ref, right_row_ref;
            double[]                result_row_ref;
            double                  cur_element;

            result.clear(left._height, right._width);
            for (int cur_row = 0; cur_row < left._height; ++cur_row)
            {
                left_row_ref   = left._contents[cur_row];
                result_row_ref =         result[cur_row];
                foreach (var cur_diagonal in left_row_ref)
                {
                    cur_element   = cur_diagonal.Value;
                    right_row_ref = right._contents[cur_diagonal.Key];
                    foreach (var cur_horizontal in right_row_ref)
                        result_row_ref[cur_horizontal.Key] += cur_element * cur_horizontal.Value;
                }
            }
        }

        static public void multiply(ref dense_matrix result, dense_matrix left, sparse_matrix right)
        {
            int left_height = left.rows, left_width = left.columns;

            if (left_width != right._height)
                throw new ArgumentException("Operand size mismatch");

            Dictionary<int, double> right_row_ref;
            double[]                left_row_ref, result_row_ref;
            double                  cur_element;

            result.clear(left_height, right._width);
            for (int cur_row = 0; cur_row < left_height; ++cur_row)
            {
                left_row_ref   =   left[cur_row];
                result_row_ref = result[cur_row];
                for (int sum_index = 0; sum_index < left_width; ++sum_index)
                {
                    cur_element   =    left_row_ref[sum_index];
                    right_row_ref = right._contents[sum_index];
                    foreach (var cur_horizontal in right_row_ref)
                        result_row_ref[cur_horizontal.Key] += cur_element * cur_horizontal.Value;
                }
            }
        }

        static public void multiply(ref dense_matrix result, sparse_matrix left, dense_matrix right)
        {
            int right_height = right.rows, right_width = right.columns;

            if (left._width != right_height)
                throw new ArgumentException("Operand size mismatch");

            Dictionary<int, double> left_row_ref;
            double[]                result_row_ref, right_row_ref;
            double                  cur_element;

            result.clear(left._height, right_width);
            for (int cur_row = 0; cur_row < left._height; ++cur_row)
            {
                left_row_ref   = left._contents[cur_row];
                result_row_ref =         result[cur_row];
                foreach (var cur_diagonal in left_row_ref)
                {
                    cur_element   = cur_diagonal.Value;
                    right_row_ref = right[cur_diagonal.Key];
                    for (int cur_column = 0; cur_column < right_width; ++cur_column)
                        result_row_ref[cur_column] += cur_element * right_row_ref[cur_column];
                }
            }
        }

        static public void slow_multiply(ref dense_matrix result, sparse_matrix left, dense_matrix right)
        {
            int right_height = right.rows, right_width = right.columns;

            if (left._width != right_height)
                throw new ArgumentException("Operand size mismatch");

            Dictionary<int, double> left_row_ref;
            double[]                result_row_ref, right_row_ref;
            double                  cur_element, product;

            //MyLog.Default.WriteLine("Width = " + right_width.ToString());
            result.clear(left._height, right_width);
            if (__neg_row == null || __neg_row.Length < right_width)
                __neg_row = new double[right_width];
            for (int cur_row = 0; cur_row < left._height; ++cur_row)
            {
                left_row_ref   = left._contents[cur_row];
                result_row_ref =         result[cur_row];
                Array.Clear(__neg_row, 0, right_width);
                foreach (var cur_diagonal in left_row_ref)
                {
                    cur_element   = cur_diagonal.Value;
                    right_row_ref = right[cur_diagonal.Key];
                    for (int cur_column = 0; cur_column < right_width; ++cur_column)
                    {
                        product = cur_element * right_row_ref[cur_column];
                        if (product >= 0.0)
                            result_row_ref[cur_column] += product;
                        else
                            __neg_row[cur_column] += product;
                    }
                }
                //MyLog.Default.WriteLine(string.Format("[{0}] := {1} - {2} = {3}", cur_row, result_row_ref[0], -__neg_row[0], result_row_ref[0] + __neg_row[0]));
                for (int cur_column = 0; cur_column < right_width; ++cur_column)
                    result_row_ref[cur_column] += __neg_row[cur_column];
            }
        }

        static public bool invert(ref sparse_matrix operand, ref sparse_matrix identity)
        {
            if (operand._width != operand._height || identity._width != identity._height || operand._height != identity._height)
                throw new ArgumentException("Cannot invert non-square matrix");

            for (int cur_item = 0; cur_item < operand._height; ++cur_item)
            {
                double cur_element;
                operand._contents[cur_item].TryGetValue(cur_item, out cur_element);
                double max_element = Math.Abs(cur_element), test_element, column_element;
                int    max_row     = cur_item;

                for (int cur_row = cur_item + 1; cur_row < operand._height; ++cur_row)
                {
                    operand._contents[cur_row].TryGetValue(cur_item, out column_element);
                    test_element = Math.Abs(column_element);
                    if (test_element > max_element)
                    {
                        max_element = test_element;
                        max_row     = cur_row;
                    }
                }
                if (max_row > cur_item)
                {
                    operand.exchange_rows (cur_item, max_row);
                    identity.exchange_rows(cur_item, max_row);
                }

                double divider, multiplier;
                operand._contents[cur_item].TryGetValue(cur_item, out divider);
                if (divider != 1.0)
                {
                    if (divider >= -revised_simplex_solver.EPSILON && divider <= revised_simplex_solver.EPSILON)
                    {
                        MyLog.Default.WriteLine(string.Format("Singular column = {0}; PR = {1}", cur_item, max_row));
                        operand.log("O");
                        return false;
                    }
                    operand.divide_row(cur_item, divider);
                    operand._contents[cur_item][cur_item] = 1.0;
                    identity.divide_row(cur_item, divider);
                }
                for (int cur_row = cur_item + 1; cur_row < operand._height; ++cur_row)
                {
                    if (operand._contents[cur_row].TryGetValue(cur_item, out multiplier))
                    {
                        operand.subtract_row(cur_row, cur_item, multiplier);
                        operand._contents[cur_row].Remove(cur_item);
                        identity.subtract_row(cur_row, cur_item, multiplier);
                    }
                }
            }

            for (int cur_item = operand._height - 1; cur_item > 0; --cur_item)
            {
                double multiplier;

                for (int cur_row = 0; cur_row < cur_item; ++cur_row)
                {
                    if (operand._contents[cur_row].TryGetValue(cur_item, out multiplier))
                    {
                        operand._contents[cur_row].Remove(cur_item);
                        identity.subtract_row(cur_row, cur_item, multiplier);
                    }
                }
            }

            sparse_matrix temp = operand;
            operand            = identity;
            identity           = temp;
            return true;
        }

        #endregion

        #region Row operations

        public void subtract_row(int destination_row, int source_row, double multiplier)
        {
            Dictionary<int, double> dest_row_ref = _contents[destination_row], source_row_ref = _contents[source_row];
            int                     cur_column;

            foreach (var cur_element in source_row_ref)
            {
                cur_column = cur_element.Key;
                if (dest_row_ref.ContainsKey(cur_column))
                    dest_row_ref[cur_column] -= cur_element.Value * multiplier;
                else
                    dest_row_ref.Add(cur_column, -cur_element.Value * multiplier);
            }
        }

        public void divide_row(int row, double divider)
        {
            Dictionary<int, double> row_ref      = _contents[row];
            int                     num_elements = 0;
            
            foreach (int cur_column in row_ref.Keys)
                __index_array[num_elements++] = cur_column;
            for (int cur_column = 0; cur_column < num_elements; ++cur_column)
                row_ref[__index_array[cur_column]] /= divider;
        }

        #endregion

        public void log(string name, bool show_contents = true)
        {
            StringBuilder row        = new StringBuilder();
            uint          non_zeroes = 0;

            MyLog.Default.WriteLine(string.Format("\n{0} {1}x{2}", name, _height, _width));
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                Dictionary<int, double> cur_row_ref = _contents[cur_row];

                row.Clear();
                row.Append("|");
                for (int cur_column = 0; cur_column < _width; ++cur_column)
                {
                    row.Append(string.Format("{0,9:G3}{1}", cur_row_ref.ContainsKey(cur_column) ? cur_row_ref[cur_column] : 0.0, cur_row_ref.ContainsKey(cur_column) ? ' ' : '*'));
                    if (cur_row_ref.ContainsKey(cur_column))
                        ++non_zeroes;
                }
                row.Append(" |");
                if (show_contents)
                    MyLog.Default.WriteLine(row.ToString());
            }
            MyLog.Default.WriteLine(string.Format("Fill = {0} %", (non_zeroes * 100) / (_width * _height)));
        }
    }
}