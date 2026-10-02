/* nRF52840 Supermini (nice!nano v2 compatible), under the Adafruit nRF52
   UF2 bootloader.
 *
 * The bootloader owns the bottom of flash — MBR at 0x0, then the SoftDevice
 * region — and hands the application 0x26000 upwards. That is the same offset
 * ZMK's `nice_nano_v2` board definition uses for its `code_partition`, and it
 * is what the UF2 file has to be based at (`tools/uf2.py --base`). Getting it
 * wrong does not fail loudly: the board takes the UF2 and then never enumerates.
 *
 * Verify on your own board: double-tap reset, then read INFO_UF2.TXT on the
 * drive that appears. If it names a different application start, change both
 * this ORIGIN and the `--base` in `.cargo/config.toml`.
 *
 * The length stops at 0xEC000 rather than at the bootloader itself, leaving the
 * bootloader's settings and storage pages alone.
 *
 * RAM starts eight bytes in because the MBR keeps its interrupt-forwarding
 * address at the very bottom of RAM. Those eight bytes cost nothing and
 * skipping them is what stops a stray write from breaking interrupt dispatch.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 0xC6000
  RAM   : ORIGIN = 0x20000008, LENGTH = 0x3FFF8
}
