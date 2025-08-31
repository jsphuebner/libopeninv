/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2025 David J. Fiddes <D.J@fiddes.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linbus.h>
#include "test.h"

class LinBusTest : public UnitTest
{
public:
   LinBusTest(const std::list<VoidFunction> *cases) : UnitTest(cases) {}
};

static void TestProtectedIdentifierGeneration()
{
   // Examples from 2.8.2 TABLE OF VALID FRAME IDENTIFIERS in
   // Lin Protocol Specification Revision 2.1

   ASSERT(LinBus::Parity(0x00) == 0x80);
   ASSERT(LinBus::Parity(0x17) == 0x97);
   ASSERT(LinBus::Parity(0x2B) == 0x2B);
   ASSERT(LinBus::Parity(0x3B) == 0xFB);
}

static void TestSpecChecksumExample()
{
   // Example from 2.8.2 EXAMPLE OF CHECKSUM CALCULATION in
   // Lin Protocol Specification Revision 2.1

   uint8_t data[] = {0x55, 0x93, 0xe5};
   ASSERT(LinBus::Checksum(0x4a, data, sizeof(data)) == 0xe6);
}

static void TestSyntheticLinStatusResponse1()
{
   // Synthesized using https://github.com/gicking/LIN_slave_portable_Arduino
   // captured and verified using Saleae Logic analyzer
   // and https://linchecksumcalculator.machsystems.cz/

   uint8_t data[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
   ASSERT(LinBus::Checksum(LinBus::Parity(0x30), data, sizeof(data)) == 0x72);
}

static void TestSyntheticLinStatusResponse2()
{
   // Synthesized using https://linchecksumcalculator.machsystems.cz/

   uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0xaa, 0x55, 0xaa, 0x55};
   ASSERT(LinBus::Checksum(0xf0, data, sizeof(data)) == 0x05);
}

static void TestVAGPTCHeaterStatusResponse1()
{
   // Chosen at random from
   // https://raw.githubusercontent.com/Tom-evnut/ID3-LIN-Bus/refs/heads/main/Lin%20Logs/PTC%20Cabin%20Heat%20on%20off.csv

   uint8_t data[] = {0x54, 0x69, 0x55, 0x00, 0xCA, 0x10, 0x00, 0x04};

   ASSERT(LinBus::Checksum(LinBus::Parity(0x0B), data, sizeof(data)) == 0x82);
}

static void TestVAGPTCHeaterStatusResponse2()
{
   // Chosen at random from
   // https://raw.githubusercontent.com/Tom-evnut/ID3-LIN-Bus/refs/heads/main/Lin%20Logs/PTC%20Cabin%20Heat%20on%20off.csv

   uint8_t data[] = {0xFF, 0x0F, 0x80, 0x64, 0x8B, 0xC9, 0xDD, 0x47};

   ASSERT(LinBus::Checksum(LinBus::Parity(0x37), data, sizeof(data)) == 0x5A);
}

// This line registers the test
REGISTER_TEST(
    LinBusTest,
    TestProtectedIdentifierGeneration,
    TestSpecChecksumExample,
    TestSyntheticLinStatusResponse1,
    TestVAGPTCHeaterStatusResponse1,
    TestVAGPTCHeaterStatusResponse2,
    TestSyntheticLinStatusResponse2);
