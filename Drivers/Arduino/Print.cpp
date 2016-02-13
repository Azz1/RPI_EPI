/*
 * Print.cpp - Base class that provides print() and println()
 * Copyright (c) 2008 David A. Mellis.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Modified 23 November 2006 by David A. Mellis
 * Modified 12 April 2011 by Marti Bolivar <mbolivar@leaflabs.com>
 */

#include "Print.h"

#include <limits.h>
#include <math.h>

#ifndef LLONG_MAX
/*
 * Note:
 *
 * At time of writing (12 April 2011), the limits.h that came with the
 * newlib we distributed didn't include LLONG_MAX.  Because we're
 * staying away from using templates (see /notes/coding_standard.rst,
 * "Language Features and Compiler Extensions"), this value was
 * copy-pasted from a println() of the value
 *
 *     std::numeric_limits<long long>::max().
 */
#define LLONG_MAX 9223372036854775807LL
#endif

/*
 * Public methods
 */
size_t Print::write(const uint8_t *buffer, uint32_t size) {
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

size_t Print::print(uint8_t b, int base) {
    return print((uint64_t)b, base);
}

size_t Print::print(const String &s)
{
  size_t n = 0;
  for (uint16_t i = 0; i < s.length(); i++) {
    n += write(s[i]);
  }
  return n;
}

size_t Print::print(char c) {
    return write(c);
}

size_t Print::print(const char str[]) {
    return write(str);
}

size_t Print::print(int n, int base) {
    return print((long long)n, base);
}

size_t Print::print(unsigned int n, int base) {
    return print((unsigned long long)n, base);
}

size_t Print::print(long n, int base) {
    return print((long long)n, base);
}

size_t Print::print(unsigned long n, int base) {
    return print((unsigned long long)n, base);
}

size_t Print::print(long long n, int base) {
    if (base == BYTE) {
        return write((uint8_t)n);
    }
    else if (base == 10) {
        if (n < 0) {
            int t = print('-');
            n = -n;
            return printNumber(n, 10) + t;
        }
        return printNumber(n, 10);
    } else {
        return printNumber(n, base);
    }
}

size_t Print::print(unsigned long long n, int base) {
    if (base == BYTE) {
        return write((uint8_t)n);
    } else {
        return printNumber(n, base);
    }
}

size_t Print::print(double n, int digits) {
    return printFloat(n, digits);
}

size_t Print::println(void) {
    size_t n = print('\r');
    n += print('\n');
    return n;
}

size_t Print::println(const String &s)
{
  size_t n = print(s);
  n += println();
  return n;
}

size_t Print::println(char c) {
    size_t n = print(c);
    n += println();
    return n;
}

size_t Print::println(const char c[]) {
    size_t n = print(c);
    n += println();
    return n;
}

size_t Print::println(uint8_t b, int base) {
    size_t n = print(b, base);
    n += println();
    return n;
}

size_t Print::println(int num, int base) {
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(unsigned int num, int base) {
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(long num, int base) {
    size_t n = print((long long)num, base);
    n += println();
    return n;
}

size_t Print::println(unsigned long num, int base) {
    size_t n = print((unsigned long long)num, base);
    n += println();
    return n;
}

size_t Print::println(long long num, int base) {
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(unsigned long long num, int base) {
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(double num, int digits) {
    size_t n = print(num, digits);
    n += println();
    return n;
}

/*
 * Private methods
 */
size_t Print::printNumber(unsigned long long n, uint8_t base) {
    char buf[CHAR_BIT * sizeof(long long) + 1];
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    if (base < 2) base = 10;

    do {
        unsigned long m = n;
        n /= base;
        char c = m - base * n;
        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n);

    return write(str);
}

/* According to snprintf(),
 *
 * nextafter((double)numeric_limits<long long>::max(), 0.0) ~= 9.22337e+18
 *
 * This slightly smaller value was picked semi-arbitrarily. */
#define LARGE_DOUBLE_TRESHOLD (9.1e18)

/* THIS FUNCTION SHOULDN'T BE USED IF YOU NEED ACCURATE RESULTS.
 *
 * This implementation is meant to be simple and not occupy too much
 * code size.  However, printing floating point values accurately is a
 * subtle task, best left to a well-tested library function.
 *
 * See Steele and White 2003 for more details:
 *
 * http://kurtstephens.com/files/p372-steele.pdf
 */
size_t Print::printFloat(double number, uint8_t digits) {
    // Hackish fail-fast behavior for large-magnitude doubles
    size_t n = 0;

    if (isnan(number)) return print("nan");
    if (isinf(number)) return print("inf");
    if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
    if (number <-4294967040.0) return print ("ovf");  // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
    	n += print('-');
        number = -number;
    }

    // Simplistic rounding strategy so that e.g. print(1.999, 2), prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) {
        rounding /= 10.0;
    }
    number += rounding;

    // Extract the integer part of the number and print it
    long long int_part = (long long)number;
    double remainder = number - (double)int_part;
    n += print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
    	n += print(".");
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int to_print = (int)remainder;
        n += print(to_print);
        remainder -= to_print;
    }

    return n;
}
