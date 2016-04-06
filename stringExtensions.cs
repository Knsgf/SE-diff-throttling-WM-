using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ttdtwm
{
    static class stringAndStringBuilderExtensions
    {
        private static readonly string m_RCSTag  = "[THR]";
        private static readonly string m_STATTag = "[STAT]";
        private static readonly string m_STABTag = "[STAB]";
        private static readonly int[]  m_RCSShiftTable, m_STATShiftTable, m_STABShiftTable;

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

        static stringAndStringBuilderExtensions()
        {
            m_RCSShiftTable  = GenerateShiftTable(m_RCSTag);
            m_STATShiftTable = GenerateShiftTable(m_STATTag);
            m_STABShiftTable = GenerateShiftTable(m_STABTag);
        }

        #endregion

        public static bool ContainsRCSTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_RCSTag, m_RCSShiftTable);
        }

        public static bool ContainsSTATTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_STATTag, m_STATShiftTable);
        }

        public static bool ContainsSTABTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_STABTag, m_STABShiftTable);
        }

        public static void ToUpperTo(this string source, StringBuilder destination)
        {
            int sourceLength = source.Length;

            destination.Length = sourceLength;
            for (int index = 0; index < sourceLength; ++index)
                destination[index] = char.ToUpper(source[index]);
        }
    }
}
