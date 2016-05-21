using System.Text;

namespace ttdtwm
{
    static class stringAndStringBuilderExtensions
    {
        private static readonly string m_THRTag     = "[THR]";
        private static readonly string m_RCSTag     = "[RCS]";
        private static readonly string m_STATTag    = "[STAT]";
        private static readonly string m_LANDINGTag = "[LANDING]";
        private static readonly string m_COTTag     = "[COT]";
        private static readonly int[]  m_THRShiftTable, m_RCSShiftTable, m_STATShiftTable, m_LANDINGShiftTable, m_COTShiftTable;

        private static StringBuilder m_blockName = new StringBuilder();

        #region Private methods

        private static int[] GenerateShiftTable(string tag)
        {
            int[] shiftTable = new int[char.MaxValue];

            for (int index = 0; index < char.MaxValue; ++index)
                shiftTable[index] = tag.Length;
            for (int index = 0; index < tag.Length - 1; ++index)
                shiftTable[tag[index]] = tag.Length - index - 1;
            return shiftTable;
        }

        private static bool ContainsTag(StringBuilder source, string tag, int[] tagShiftTable)
        {
            int sourceLength = source.Length, startIndex = 0, endIndex = tag.Length - 1, tagIndex, shift;
            bool matchFound;

            while (endIndex < sourceLength)
            {
                matchFound = true;
                tagIndex   = 0;
                for (int index = startIndex; index <= endIndex; ++index)
                {
                    if (tag[tagIndex++] != source[index])
                    {
                        matchFound = false;
                        break;
                    }
                }
                if (matchFound)
                    return true;
                shift       = tagShiftTable[source[endIndex]];
                startIndex += shift;
                endIndex   += shift;
            }
            return false;
        }

        private static string AddThrusterTag(string source, string tag)
        {
            string result = source.Trim();
            int    left_bracket_pos = result.LastIndexOf('('), right_bracket_pos = result.LastIndexOf(')');

            if (left_bracket_pos >= 0 && right_bracket_pos == result.Length - 1)
                result = result.Substring(0, left_bracket_pos).TrimEnd();
            return tag + result;
        }

        private static string RemoveTag(string source, string tag, bool is_thruster_tag)
        {
            if (source.Length < tag.Length)
                return source;

            string result = source, source_upper = source.ToUpper();
            for (int position = source_upper.IndexOf(tag); position >= 0; position = source_upper.IndexOf(tag))
            {
                int  remove_position = position, chars_to_remove = tag.Length;

                bool add_space = remove_position > 0 && char.IsWhiteSpace(source_upper[remove_position - 1]) || remove_position + chars_to_remove < source_upper.Length && char.IsWhiteSpace(source_upper[remove_position + chars_to_remove + 1]);
                source_upper   = source_upper.Substring(0, remove_position).TrimEnd() + (add_space ? " " : "") + source_upper.Substring(remove_position + chars_to_remove).TrimStart();
                result         =       result.Substring(0, remove_position).TrimEnd() + (add_space ? " " : "") +       result.Substring(remove_position + chars_to_remove).TrimStart();
            }

            result = result.Trim();
            if (is_thruster_tag)
            {
                int left_bracket_pos = result.LastIndexOf('('), right_bracket_pos = result.LastIndexOf(')');
                if (left_bracket_pos >= 0 && right_bracket_pos == result.Length - 1)
                    result = result.Substring(0, left_bracket_pos).TrimEnd();
            }

            return result;
        }

        static stringAndStringBuilderExtensions()
        {
            m_THRShiftTable     = GenerateShiftTable(m_THRTag    );
            m_RCSShiftTable     = GenerateShiftTable(m_RCSTag    );
            m_STATShiftTable    = GenerateShiftTable(m_STATTag   );
            m_LANDINGShiftTable = GenerateShiftTable(m_LANDINGTag);
            m_COTShiftTable     = GenerateShiftTable(m_COTTag    );
        }

        #endregion

        #region ContainsTag()

        public static bool ContainsTHRTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_THRTag, m_THRShiftTable);
        }

        public static bool ContainsRCSTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_RCSTag, m_RCSShiftTable);
        }

        public static bool ContainsSTATTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_STATTag, m_STATShiftTable);
        }

        public static bool ContainsLANDINGTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_LANDINGTag, m_LANDINGShiftTable);
        }

        public static bool ContainsCOTTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_COTTag, m_COTShiftTable);
        }

        #endregion

        #region AddTag()

        public static string AddTHRTag(this string blockName)
        {
            blockName.ToUpperTo(m_blockName);
            return m_blockName.ContainsTHRTag() ? blockName : AddThrusterTag(blockName, m_THRTag);
        }

        public static string AddRCSTag(this string blockName)
        {
            blockName.ToUpperTo(m_blockName);
            return m_blockName.ContainsRCSTag() ? blockName : AddThrusterTag(blockName, m_RCSTag);
        }

        public static string AddSTATTag(this string blockName)
        {
            blockName.ToUpperTo(m_blockName);
            return m_blockName.ContainsSTATTag() ? blockName : AddThrusterTag(blockName, m_STATTag);
        }

        public static string AddLANDINGTag(this string blockName)
        {
            blockName.ToUpperTo(m_blockName);
            return m_blockName.ContainsLANDINGTag() ? blockName : (m_LANDINGTag + blockName);
        }

        public static string AddCOTTag(this string blockName)
        {
            blockName.ToUpperTo(m_blockName);
            return m_blockName.ContainsCOTTag() ? blockName : (m_COTTag + blockName);
        }

        #endregion

        #region RemoveTag()

        public static string RemoveTHRTag(this string blockName)
        {
            return RemoveTag(blockName, m_THRTag, is_thruster_tag: true);
        }

        public static string RemoveRCSTag(this string blockName)
        {
            return RemoveTag(blockName, m_RCSTag, is_thruster_tag: true);
        }

        public static string RemoveSTATTag(this string blockName)
        {
            return RemoveTag(blockName, m_STATTag, is_thruster_tag: true);
        }

        public static string RemoveLANDINGTag(this string blockName)
        {
            return RemoveTag(blockName, m_LANDINGTag, is_thruster_tag: false);
        }

        public static string RemoveCOTTag(this string blockName)
        {
            return RemoveTag(blockName, m_COTTag, is_thruster_tag: false);
        }

        #endregion

        public static void ToUpperTo(this string source, StringBuilder destination)
        {
            int sourceLength = source.Length;

            destination.Length = sourceLength;
            for (int index = 0; index < sourceLength; ++index)
                destination[index] = char.ToUpper(source[index]);
        }
    }
}
