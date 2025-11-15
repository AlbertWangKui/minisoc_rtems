
#include <osp_mmap.h>

#include <libcpu/arm-cp15.h>

unsigned int ospSetTranslationTableEntries(const void *begin,const void *end,unsigned section_flags)
{
    return arm_cp15_set_translation_table_entries(begin,end,section_flags);
}
