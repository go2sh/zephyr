. = ALIGN(256);
__btv = .;
KEEP(*(.vectors.trap*))
__biv = .;
KEEP(*(SORT_BY_NAME(.vectors.irq*)))