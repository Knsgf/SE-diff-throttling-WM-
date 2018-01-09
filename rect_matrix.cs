using System;
using System.Text;

using VRage.Utils;

namespace ttdtwm
{
    struct dense_matrix
    {
        private int        _width, _initial_width, _height;
        private double[][] _contents;

        public double[] this[int row]
        {
            get
            {
                return _contents[row];
            }
        }

        public int rows
        {
            get
            {
                return _height;
            }
        }

        public int columns
        {
            get
            {
                return _width;
            }
        }

        #region Assignment operations

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

            if (new_rows == _height && new_columns == _initial_width)
            {
                _width = _initial_width;
                for (int cur_row = 0; cur_row < _height; ++cur_row)
                    Array.Clear(_contents[cur_row], 0, _width);
            }
            else
            {
                _width    = _initial_width = new_columns;
                _height   = new_rows;
                _contents = new double[_height][];
                for (int cur_row = 0; cur_row < _height; ++cur_row)
                    _contents[cur_row] = new double[_width];
            }
        }

        /*
        public void set_to_identity(int size = 0)
        {
            clear(size);
            for (int cur_element = 0; cur_element < _width; ++cur_element)
                _contents[cur_element][cur_element] = 1.0;
        }

        public void copy_from(dense_matrix b)
        {
            if (_width != b._width || _height != b._height)
                clear(b._height, b._width);

            double[] cur_row_ref, b_row_ref;
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                cur_row_ref =   _contents[cur_row];
                b_row_ref   = b._contents[cur_row];
                for (int cur_column = 0; cur_column < _width; ++cur_column)
                    cur_row_ref[cur_column] = b_row_ref[cur_column];
            }
        }
        */

        public void column_vector_from(dense_matrix b, int column)
        {
            if (_width != 1 || _height != b._height)
                clear(b._height, 1);

            for (int cur_row = 0; cur_row < _height; ++cur_row)
                _contents[cur_row][0] = b._contents[cur_row][column];
        }

        /*
        public static void exchange_columns(dense_matrix a, int column_a, dense_matrix b, int column_b)
        {
            if (a._height != b._height)
                throw new ArgumentException("Row number mismatch");

            double temp;
            for (int cur_row = 0; cur_row < a._height; ++cur_row)
            {
                temp                           = a._contents[cur_row][column_a];
                a._contents[cur_row][column_a] = b._contents[cur_row][column_b];
                b._contents[cur_row][column_b] = temp;
            }
        }
        */

        public void shrink_width(int amount)
        {
            if (amount < 0 || _width <= amount)
                throw new ArgumentException("Shrink amount negative or too big");
            _width -= amount;
        }

        public void zero_roundoff_errors()
        {
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                double[] row_ref = _contents[cur_row];

                for (int cur_column = 0; cur_column < _width; ++cur_column)
                {
                    if (row_ref[cur_column] > -revised_simplex_solver.EPSILON && row_ref[cur_column] < revised_simplex_solver.EPSILON)
                        row_ref[cur_column] = 0.0;
                }
            }
        }

        #endregion

        #region Arithmetic

        /*
        public void add(rect_matrix b)
        {
            if (_width != b._width || _height != b._height)
                throw new ArgumentException("Attempt to add matrix of different size");

            double[] cur_row_ref, b_row_ref;
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                cur_row_ref =   _contents[cur_row];
                b_row_ref   = b._contents[cur_row];
                for (int cur_column = 0; cur_column < _width; ++cur_column)
                    cur_row_ref[cur_column] += b_row_ref[cur_column];
            }
        }

        public void subtract(dense_matrix b)
        {
            if (_width != b._width || _height != b._height)
                throw new ArgumentException("Attempt to subtract matrix of different size");

            double[] cur_row_ref, b_row_ref;
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                cur_row_ref =   _contents[cur_row];
                b_row_ref   = b._contents[cur_row];
                for (int cur_column = 0; cur_column < _width; ++cur_column)
                    cur_row_ref[cur_column] -= b_row_ref[cur_column];
            }
        }

        static public void multiply(ref dense_matrix result, dense_matrix left, dense_matrix right)
        {
            if (left._width != right._height)
                throw new ArgumentException("Operand size mismatch");

            double[] result_row_ref, left_row_ref, right_row_ref;
            double   left_element;

            result.clear(left._height, right._width);
            for (int cur_row = 0; cur_row < result._height; ++cur_row)
            {
                left_row_ref   =   left._contents[cur_row];
                result_row_ref = result._contents[cur_row];
                for (int sum_index = 0; sum_index < left._width; ++sum_index)
                {
                    right_row_ref = right._contents[sum_index];
                    left_element  =    left_row_ref[sum_index];
                    for (int cur_column = 0; cur_column < result._width; ++cur_column)
                        result_row_ref[cur_column] += left_element * right_row_ref[cur_column];
                }
            }
        }

        static public bool invert(ref dense_matrix operand, ref dense_matrix identity)
        {
            if (operand._width != operand._height || identity._width != identity._height || operand._height != identity._height)
                throw new ArgumentException("Cannot invert non-square matrix");

            for (int cur_item = 0; cur_item < operand._height; ++cur_item)
            {
                double cur_element = operand._contents[cur_item][cur_item];
                double max_element = Math.Abs(cur_element), test_element;
                int    max_row     = cur_item;

                //operand.display(identity);
                for (int cur_row = cur_item + 1; cur_row < operand._height; ++cur_row)
                {
                    test_element = Math.Abs(operand._contents[cur_row][cur_item] + cur_element);
                    if (test_element > max_element)
                    {
                        max_element = test_element;
                        max_row     = cur_row;
                    }
                }
                if (max_row > cur_item)
                {
                    operand.add_row (cur_item, max_row, cur_item);
                    identity.add_row(cur_item, max_row);
                }

                //operand.display(identity);
                double divider = operand._contents[cur_item][cur_item], multiplier;
                if (divider > -EPSILON && divider < EPSILON)
                    return false;
                operand._contents[cur_item][cur_item] = 1.0;
                operand.divide_row (cur_item, divider, cur_item + 1);
                identity.divide_row(cur_item, divider);
                //operand.display(identity);
                for (int cur_row = cur_item + 1; cur_row < operand._height; ++cur_row)
                {
                    multiplier = operand._contents[cur_row][cur_item];
                    operand._contents[cur_row][cur_item] = 0.0;
                    operand.subtract_row (cur_row, cur_item, multiplier, cur_item + 1);
                    identity.subtract_row(cur_row, cur_item, multiplier);
                }
            }

            for (int cur_item = operand._height - 1; cur_item > 0; --cur_item)
            {
                for (int cur_row = 0; cur_row < cur_item; ++cur_row)
                {
                    identity.subtract_row(cur_row, cur_item, operand._contents[cur_row][cur_item]);
                    operand._contents[cur_row][cur_item] = 0.0;
                }
            }

            dense_matrix temp = operand;
            operand          = identity;
            identity         = temp;
            return true;
        }

        */
        #endregion

        #region row operations

        /*
        public void add_row(int destination_row, int source_row, int starting_index = 0)
        {
            double[] dest_row_ref = _contents[destination_row], source_row_ref = _contents[source_row];

            for (int cur_column = starting_index; cur_column < _width; ++cur_column)
                dest_row_ref[cur_column] += source_row_ref[cur_column];
        }

        public void subtract_row(int destination_row, int source_row, double multiplier, int starting_index = 0)
        {
            double[] dest_row_ref = _contents[destination_row], source_row_ref = _contents[source_row];

            for (int cur_column = starting_index; cur_column < _width; ++cur_column)
            {
                dest_row_ref[cur_column] -= source_row_ref[cur_column] * multiplier;
                if (dest_row_ref[cur_column] > -EPSILON && dest_row_ref[cur_column] < EPSILON)
                    dest_row_ref[cur_column] = 0.0;
            }
        }

        public void divide_row(int row, double divider, int starting_index = 0)
        {
            double[] row_ref = _contents[row];

            for (int cur_column = starting_index; cur_column < _width; ++cur_column)
                row_ref[cur_column] /= divider;
        }
        */

        #endregion

        public void log(string name, bool show_contents = true)
        {
            StringBuilder row = new StringBuilder();

            MyLog.Default.WriteLine(string.Format("\n{0} {1}x{2}", name, _height, _width));
            if (!show_contents)
                return;
            for (int cur_row = 0; cur_row < _height; ++cur_row)
            {
                double[] cur_row_ref = _contents[cur_row];

                row.Clear();
                row.Append("|");
                for (int cur_column = 0; cur_column < _width; ++cur_column)
                    row.Append(string.Format("{0,10:G3}", cur_row_ref[cur_column]));
                row.Append(" |");
                MyLog.Default.WriteLine(row.ToString());
            }
        }
    }
}
