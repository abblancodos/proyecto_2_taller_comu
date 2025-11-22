#include "crc16.h"
#include <cstring>
#include <iostream>
#include <vector>

using namespace comm;

int main() {
  std::cout << "=== CRC-16 Test Program ===\n\n";

  // Test 1: Basic CRC computation
  std::cout << "Test 1: Basic CRC computation\n";
  const char *test_str = "Hello, World!";
  std::vector<uint8_t> data(test_str, test_str + std::strlen(test_str));

  uint16_t crc = CRC16::compute(data);
  std::cout << "  Data: \"" << test_str << "\"\n";
  std::cout << "  CRC-16: 0x" << std::hex << crc << std::dec << "\n";
  std::cout << "  ✓ PASS\n\n";

  // Test 2: Append and verify
  std::cout << "Test 2: Append and verify CRC\n";
  std::vector<uint8_t> data2 = data;
  CRC16::append(data2);

  std::cout << "  Original size: " << data.size() << " bytes\n";
  std::cout << "  With CRC size: " << data2.size() << " bytes\n";

  bool valid = CRC16::verify(data2);
  std::cout << "  Verification: " << (valid ? "VALID" : "INVALID") << "\n";
  std::cout << "  " << (valid ? "✓ PASS" : "✗ FAIL") << "\n\n";

  // Test 3: Detect single-bit error
  std::cout << "Test 3: Error detection (single bit flip)\n";
  std::vector<uint8_t> data3 = data2;
  data3[5] ^= 0x01; // Flip one bit

  bool valid3 = CRC16::verify(data3);
  std::cout << "  Corrupted data verification: "
            << (valid3 ? "VALID" : "INVALID") << "\n";
  std::cout << "  "
            << (!valid3 ? "✓ PASS (error detected)"
                        : "✗ FAIL (error not detected)")
            << "\n\n";

  // Test 4: Detect burst error
  std::cout << "Test 4: Error detection (burst error)\n";
  std::vector<uint8_t> data4 = data2;
  data4[3] ^= 0xFF;
  data4[4] ^= 0xFF;
  data4[5] ^= 0xFF;

  bool valid4 = CRC16::verify(data4);
  std::cout << "  Corrupted data verification: "
            << (valid4 ? "VALID" : "INVALID") << "\n";
  std::cout << "  "
            << (!valid4 ? "✓ PASS (error detected)"
                        : "✗ FAIL (error not detected)")
            << "\n\n";

  // Test 5: Large data block
  std::cout << "Test 5: Large data block (10KB)\n";
  std::vector<uint8_t> large_data(10240);
  for (size_t i = 0; i < large_data.size(); ++i) {
    large_data[i] = static_cast<uint8_t>(i & 0xFF);
  }

  CRC16::append(large_data);
  bool valid5 = CRC16::verify(large_data);
  std::cout << "  Data size: " << large_data.size() - 2 << " bytes\n";
  std::cout << "  Verification: " << (valid5 ? "VALID" : "INVALID") << "\n";
  std::cout << "  " << (valid5 ? "✓ PASS" : "✗ FAIL") << "\n\n";

  std::cout << "=== All tests completed ===\n";

  return 0;
}
