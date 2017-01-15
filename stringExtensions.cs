using System;
using System.Text;

namespace ttdtwm
{
    static class stringExtensions
    {
        const int THR_MASK = 0x1, SLP_MASK = 0x2, STAT_MASK = 0x4;
        const int COT_MASK = 0x1, LANDING_MASK = 0x2;

        private static readonly string m_Prefix  = "[TP&DT2: ";
        private static readonly string m_Suffix  = "]\n";
        private static readonly int[]  m_PrefixShiftTable;
        private static readonly int    m_PrefixLength;

        private static readonly string m_OldPrefix = "[TP&DT: ";
        private static readonly int[]  m_OldPrefixShiftTable;
        private static readonly int    m_OldPrefixLength;

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

        static stringExtensions()
        {
            m_PrefixShiftTable = GenerateShiftTable(m_Prefix);
            m_PrefixLength     = m_Prefix.Length;

            m_OldPrefixShiftTable = GenerateShiftTable(m_OldPrefix);
            m_OldPrefixLength     = m_OldPrefix.Length;
        }

        private static int? TagPosition(string source, string tag, int[] tagShiftTable)
        {
            int sourceLength = source.Length, startIndex = 0, endIndex = tag.Length - 1, tagIndex, shift;
            bool matchFound;

            while (endIndex < sourceLength)
            {
                matchFound = true;
                tagIndex = 0;
                for (int index = startIndex; index <= endIndex; ++index)
                {
                    if (tag[tagIndex++] != source[index])
                    {
                        matchFound = false;
                        break;
                    }
                }
                if (matchFound)
                    return startIndex;
                shift = tagShiftTable[source[endIndex]];
                startIndex += shift;
                endIndex += shift;
            }
            return null;
        }

        private static char ConvertValueToChar(int value)
        {
            if (value >= 0)
            {
                if (value < 10)
                    return (char) (value + '0');
                if (value < 36)
                    return (char) (value + 'A' - 10);
                if (value < 62)
                    return (char) (value + 'a' - 36);
                switch (value)
                {
                    case 62:
                        return '+';

                    case 63:
                        return '-';
                }
            }
            throw new Exception("Tag value must be between 0 and 63 inclusive");
        }

        private static int ConvertCharToValue(char symbol)
        {
            if (symbol >= '0' && symbol <= '9')
                return symbol - '0';
            if (symbol >= 'A' && symbol <= 'Z')
                return symbol + 10 - 'A';
            if (symbol >= 'a' && symbol <= 'z')
                return symbol + 36 - 'a';
            switch (symbol)
            {
                case '+':
                    return 62;

                case '-':
                    return 63;
            }
            return 0;
        }

        private static string RemoveOldTag(string source)
        {
            int? tagPosition = TagPosition(source, m_OldPrefix, m_OldPrefixShiftTable);

            if (tagPosition == null)
                return source;

            int    tagLength = m_OldPrefixLength + 1 + m_Suffix.Length;
            int    tagIndex  = (int) tagPosition;
            string result    = (tagIndex > 0) ? source.Substring(0, tagIndex) : "";
            if (tagIndex + tagLength < source.Length)
                result += source.Substring(tagIndex + tagLength);
            return result;
        }

        private static string SetTagValue(string source, int newValue)
        {
            string temp      = RemoveOldTag(source);
            int? tagPosition = TagPosition(temp, m_Prefix, m_PrefixShiftTable);

            if (tagPosition == null)
                return m_Prefix + ConvertValueToChar(newValue) + m_Suffix + temp;
            int    tagIndex = (int) tagPosition + m_PrefixLength;
            string result   = temp.Substring(0, tagIndex) + ConvertValueToChar(newValue);
            if (temp.Length > tagIndex + 1)
                result += temp.Substring(tagIndex + 1);
            return result;
        }

        private static int GetTagValue(string source)
        {
            const int RCS_MASK = 0x2;

            int? tagPosition = TagPosition(source, m_Prefix, m_PrefixShiftTable);
            bool isOldTag     = false;

            if (tagPosition == null)
            {
                tagPosition = TagPosition(source, m_OldPrefix, m_OldPrefixShiftTable);
                if (tagPosition == null)
                    return 0;
                isOldTag = true;
            }
            int tagIndex = (int) tagPosition + (isOldTag ? m_OldPrefixLength : m_PrefixLength);
            if (source.Length <= tagIndex)
                return 0;
            if (!isOldTag)
                return ConvertCharToValue(source[tagIndex]);

            int oldValue = ConvertCharToValue(source[tagIndex]);
            if ((oldValue & THR_MASK) != 0)
                oldValue |= SLP_MASK;
            else if ((oldValue & RCS_MASK) != 0)
                oldValue = (oldValue | THR_MASK) & ~SLP_MASK;
            return oldValue;
        }

        #endregion

        #region ContainsTag()

        public static bool ContainsTHRTag(this string blockData)
        {
            int tagValue = GetTagValue(blockData);
            return (tagValue & THR_MASK) != 0 && (tagValue & SLP_MASK) != 0;
        }

        public static bool ContainsRCSTag(this string blockData)
        {
            int tagValue = GetTagValue(blockData);
            return (tagValue & THR_MASK) != 0 && (tagValue & SLP_MASK) == 0;
        }

        public static bool ContainsSLPTag(this string blockData)
        {
            return (GetTagValue(blockData) & SLP_MASK) != 0;
        }

        public static bool ContainsSTATTag(this string blockData)
        {
            int tagValue = GetTagValue(blockData);
            return (tagValue & STAT_MASK) != 0 && (tagValue & SLP_MASK) != 0;
        }

        public static bool ContainsLANDINGTag(this string blockData)
        {
            return (GetTagValue(blockData) & LANDING_MASK) != 0;
        }

        public static bool ContainsCOTTag(this string blockData)
        {
            return (GetTagValue(blockData) & COT_MASK) != 0;
        }

        #endregion

        public static string UpgradeTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData));
        }

        #region AddTag()

        public static string AddTHRTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | THR_MASK | SLP_MASK);
        }

        public static string AddRCSTag(this string blockData)
        {
            return SetTagValue(blockData, (GetTagValue(blockData) | THR_MASK) & ~SLP_MASK);
        }

        public static string AddSTATTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | STAT_MASK);
        }

        public static string AddLANDINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | LANDING_MASK);
        }

        public static string AddCOTTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | COT_MASK);
        }

        #endregion

        #region RemoveTag()

        public static string RemoveTHRTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~THR_MASK);
        }

        public static string RemoveRCSTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~THR_MASK);
        }

        public static string RemoveSTATTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~STAT_MASK);
        }

        public static string RemoveLANDINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~LANDING_MASK);
        }

        public static string RemoveCOTTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~COT_MASK);
        }

        #endregion
    }
}
