/* kern/i386/tsc.c - x86 TSC time source implementation
 * Requires Pentium or better x86 CPU that supports the RDTSC instruction.
 * This module uses the PIT to calibrate the TSC to
 * real time.
 *
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2008  Free Software Foundation, Inc.
 *
 *  GRUB is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  GRUB is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GRUB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <grub/types.h>
#include <grub/time.h>
#include <grub/misc.h>
#include <grub/i386/tsc.h>
#include <grub/i386/pmtimer.h>
#include <grub/acpi.h>
#include <grub/cpu/io.h>

static void *
grub_acpi_rsdt_find_table (struct grub_acpi_table_header *rsdt, const char *sig)
{
  grub_size_t s;
  grub_uint32_t *ptr;

  if (!rsdt)
    return 0;

  if (grub_memcmp (rsdt->signature, "RSDT", 4) != 0)
    return 0;

  ptr = (grub_uint32_t *) (rsdt + 1);
  s = (rsdt->length - sizeof (*rsdt)) / sizeof (grub_uint32_t);
  for (; s; s--, ptr++)
    {
      struct grub_acpi_table_header *tbl;
      tbl = (struct grub_acpi_table_header *) (grub_addr_t) *ptr;
      if (grub_memcmp (tbl->signature, sig, 4) == 0)
	return tbl;
    }
  return 0;
}

static void *
grub_acpi_xsdt_find_table (struct grub_acpi_table_header *xsdt, const char *sig)
{
  grub_size_t s;
  grub_uint64_t *ptr;

  if (!xsdt)
    return 0;

  if (grub_memcmp (xsdt->signature, "XSDT", 4) != 0)
    return 0;

  ptr = (grub_uint64_t *) (xsdt + 1);
  s = (xsdt->length - sizeof (*xsdt)) / sizeof (grub_uint32_t);
  for (; s; s--, ptr++)
    {
      struct grub_acpi_table_header *tbl;
#if GRUB_CPU_SIZEOF_VOID_P != 8
      if (*ptr >> 32)
	continue;
#endif
      tbl = (struct grub_acpi_table_header *) (grub_addr_t) *ptr;
      if (grub_memcmp (tbl->signature, sig, 4) == 0)
	return tbl;
    }
  return 0;
}

struct grub_acpi_fadt *
grub_acpi_find_fadt (void)
{
  struct grub_acpi_fadt *fadt = 0;
  struct grub_acpi_rsdp_v10 *rsdpv1;
  struct grub_acpi_rsdp_v20 *rsdpv2;
  rsdpv1 = grub_machine_acpi_get_rsdpv1 ();
  if (rsdpv1)
    fadt = grub_acpi_rsdt_find_table ((struct grub_acpi_table_header *)
				      (grub_addr_t) rsdpv1->rsdt_addr,
				      GRUB_ACPI_FADT_SIGNATURE);
  if (fadt)
    return fadt;
  rsdpv2 = grub_machine_acpi_get_rsdpv2 ();
  if (rsdpv2)
    fadt = grub_acpi_rsdt_find_table ((struct grub_acpi_table_header *)
				      (grub_addr_t) rsdpv2->rsdpv1.rsdt_addr,
				      GRUB_ACPI_FADT_SIGNATURE);
  if (fadt)
    return fadt;
  if (rsdpv2
#if GRUB_CPU_SIZEOF_VOID_P != 8
      && !(rsdpv2->xsdt_addr >> 32)
#endif
      )
    fadt = grub_acpi_xsdt_find_table ((struct grub_acpi_table_header *)
				      (grub_addr_t) rsdpv2->xsdt_addr,
				      GRUB_ACPI_FADT_SIGNATURE);
  if (fadt)
    return fadt;
  return 0;
}

grub_uint64_t
grub_pmtimer_wait_count_tsc (grub_port_t pmtimer,
			     grub_uint16_t num_pm_ticks)
{
  grub_uint32_t start;
  grub_uint32_t last;
  grub_uint32_t cur, end;
  grub_uint64_t start_tsc;
  grub_uint64_t end_tsc;
  int num_iter = 0;

  start = grub_inl (pmtimer) & 0xffffff;
  last = start;
  end = start + num_pm_ticks;
  start_tsc = grub_get_tsc ();
  while (1)
    {
      cur = grub_inl (pmtimer) & 0xffffff;
      if (cur < last)
	cur |= 0x1000000;
      num_iter++;
      if (cur >= end)
	{
	  end_tsc = grub_get_tsc ();
	  return end_tsc - start_tsc;
	}
      /* Check for broken PM timer.
	 50000000 TSCs is between 5 ms (10GHz) and 200 ms (250 MHz)
	 if after this time we still don't have 1 ms on pmtimer, then
	 pmtimer is broken.
       */
      if ((num_iter & 0xffffff) == 0 && grub_get_tsc () - start_tsc > 5000000) {
	return 0;
      }
    }
}

int
grub_tsc_calibrate_from_pmtimer (void)
{
  struct grub_acpi_fadt *fadt;
  grub_port_t pmtimer;
  grub_uint64_t tsc_diff;

  fadt = grub_acpi_find_fadt ();
  if (!fadt)
    return 0;
  pmtimer = fadt->pmtimer;
  if (!pmtimer)
    return 0;

  /* It's 3.579545 MHz clock. Wait 1 ms.  */
  tsc_diff = grub_pmtimer_wait_count_tsc (pmtimer, 3580);
  if (tsc_diff == 0)
    return 0;
  grub_tsc_rate = grub_divmod64 ((1ULL << 32), tsc_diff, 0);
  return 1;
}
