/*
 * Copyright (c) 2023, openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 SECTION_PROLOGUE(.resource_table,, SUBALIGN(4))
        {
          _resource_table_start = .;
          KEEP(*(.resource_table*))
          _resource_table_end = .;
        } GROUP_LINK_IN(ROMABLE_REGION)
        _resource_tables_size = _resource_table_end - _resource_table_start;
