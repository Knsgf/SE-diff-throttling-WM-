using System;

using VRageMath;

namespace ttdtwm
{
    static class stringExtensions
    {
        const int THR_MASK = 0x1,     SLP_MASK = 0x2,    STAT_MASK = 0x4, NL_MASK = 0x8;
        const int COT_MASK = 0x1, LANDING_MASK = 0x2, DAMPING_MASK = 0x4, IC_MASK = 0x8;

        private static readonly string m_Prefix  = "[TP&DT2: ";
        private static readonly string m_Suffix  = "]\n";
        private static readonly int[]  m_PrefixShiftTable;
        private static readonly int    m_PrefixLength;

        private static string m_LastSource   = null, m_LastTag = null;
        private static int?   m_LastPosition = null;

        #region Tagging

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
        }

        private static int? TagPosition(string source, string tag, int[] tagShiftTable)
        {
            if (ReferenceEquals(source, m_LastSource) && ReferenceEquals(tag, m_LastTag))
                return m_LastPosition;

            int  sourceLength = source.Length, startIndex = 0, endIndex = tag.Length - 1, tagIndex, shift;
            bool matchFound;

            m_LastSource = source;
            m_LastTag    = tag;
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
                {
                    m_LastPosition = startIndex;
                    return startIndex;
                }
                shift       = tagShiftTable[source[endIndex]];
                startIndex += shift;
                endIndex   += shift;
            }

            m_LastPosition = null;
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
                if (value < 157)
                    return (char) (value + 0xA1 - 62);
            }
            throw new ArgumentException("Tag value must be between 0 and 156 inclusive");
        }

        private static int ConvertCharToValue(char symbol)
        {
            if (symbol >= '0' && symbol <= '9')
                return symbol - '0';
            if (symbol >= 'A' && symbol <= 'Z')
                return symbol + 10 - 'A';
            if (symbol >= 'a' && symbol <= 'z')
                return symbol + 36 - 'a';
            if (symbol >= 0xA1 && symbol <= 0xFF)
                return symbol + 62 - 0xA1;
            return 0;
        }

        private static string SetTagValue(string source, int newValue)
        {
            int? tagPosition = TagPosition(source, m_Prefix, m_PrefixShiftTable);

            if (tagPosition == null)
                return m_Prefix + ConvertValueToChar(newValue) + m_Suffix + source;
            int    tagIndex = (int) tagPosition + m_PrefixLength;
            string result   = source.Substring(0, tagIndex) + ConvertValueToChar(newValue);
            if (source.Length > tagIndex + 1)
                result += source.Substring(tagIndex + 1);
            return result;
        }

        private static int GetTagValue(string source)
        {
            int? tagPosition = TagPosition(source, m_Prefix, m_PrefixShiftTable);

            if (tagPosition == null)
                return 0;
            int tagIndex = (int) tagPosition + m_PrefixLength;
            if (source.Length <= tagIndex)
                return 0;
            return ConvertCharToValue(source[tagIndex]);
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

        public static bool ContainsNLTag(this string blockData)
        {
            return (GetTagValue(blockData) & NL_MASK) != 0;
        }

        public static bool ContainsSTATTag(this string blockData)
        {
            int tagValue = GetTagValue(blockData);
            return (tagValue & STAT_MASK) != 0 && ((tagValue & THR_MASK) == 0 || (tagValue & SLP_MASK) != 0);
        }

        public static bool ContainsLANDINGTag(this string blockData)
        {
            return (GetTagValue(blockData) & LANDING_MASK) != 0;
        }

        public static bool ContainsCOTTag(this string blockData)
        {
            return (GetTagValue(blockData) & COT_MASK) != 0;
        }

        public static bool ContainsDAMPINGTag(this string blockData)
        {
            return (GetTagValue(blockData) & DAMPING_MASK) == 0;
        }

        public static bool ContainsICTag(this string blockData)
        {
            return (GetTagValue(blockData) & IC_MASK) != 0;
        }

        #endregion

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

        public static string AddNLTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | NL_MASK);
        }

        public static string AddLANDINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | LANDING_MASK);
        }

        public static string AddCOTTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | COT_MASK);
        }

        public static string AddDAMPINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~DAMPING_MASK);
        }

        public static string AddICTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | IC_MASK);
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

        public static string RemoveNLTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~NL_MASK);
        }

        public static string RemoveLANDINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~LANDING_MASK);
        }

        public static string RemoveCOTTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~COT_MASK);
        }

        public static string RemoveDAMPINGTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) | DAMPING_MASK);
        }

        public static string RemoveICTag(this string blockData)
        {
            return SetTagValue(blockData, GetTagValue(blockData) & ~IC_MASK);
        }

        #endregion

        #region ID override

        public static Vector3 IDOverrides(this string blockData)
        {
            Vector3 result   = Vector3.Zero;
            int     tagValue = GetTagValue(blockData);

            if ((tagValue & 0x10) != 0)
                result.X = 1.0f;
            if ((tagValue & 0x20) != 0)
                result.Y = 1.0f;
            if ((tagValue & 0x40) != 0)
                result.Z = 1.0f;
            return result;
        }

        public static string SetIDOvveride(this string blockData, Vector3 overrideEnable)
        {
            int overrideFlags = 0;

            if (Math.Abs(overrideEnable.X) >= 0.5f)
                overrideFlags |= 0x10;
            if (Math.Abs(overrideEnable.Y) >= 0.5f)
                overrideFlags |= 0x20;
            if (Math.Abs(overrideEnable.Z) >= 0.5f)
                overrideFlags |= 0x40;
            return SetTagValue(blockData, (GetTagValue(blockData) & 0xF) | overrideFlags);
        }

        #endregion
    }
}
