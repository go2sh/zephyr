. = ALIGN(256);
__btv = .;
KEEP(*(.vectors.trap*))
. = ALIGN(8192);
__biv = .;
KEEP(*(SORT_BY_NAME(.vectors.irq*)))
. = ALIGN(32);