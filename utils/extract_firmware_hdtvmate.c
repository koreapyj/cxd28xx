#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <elf.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

struct fw_entry {
	const char *sym_name;
	const char *out_name;
};

static const struct fw_entry fw_table[] = {
	/* IT930x bridge firmware */
	{ "brFirmware_codes",      "dvb-usb-it9306-01.fw" },
	// { "brFirmware_segments",   "brFirmware_segments.bin" },
	// { "brFirmware_partitions", "brFirmware_partitions.bin" },
	// { "brFirmware_scripts",    "brFirmware_scripts.bin" },
	// { "brFirmware_scriptSets", "brFirmware_scriptSets.bin" },
	// /* IT913x demodulator firmware */
	// { "Firmware_codes",        "Firmware_codes.bin" },
	// { "Firmware_segments",     "Firmware_segments.bin" },
	// { "Firmware_partitions",   "Firmware_partitions.bin" },
	// { "Firmware_scriptSets",   "Firmware_scriptSets.bin" },
	// { "Firmware_scripts",      "Firmware_scripts.bin" },
	// /* IT913x V2 */
	// { "FirmwareV2_codes",        "FirmwareV2_codes.bin" },
	// { "FirmwareV2_segments",     "FirmwareV2_segments.bin" },
	// { "FirmwareV2_partitions",   "FirmwareV2_partitions.bin" },
	// { "FirmwareV2_scriptSets",   "FirmwareV2_scriptSets.bin" },
	// { "FirmwareV2_scripts",      "FirmwareV2_scripts.bin" },
	// /* IT913x V2I (ISDB-T) */
	// { "FirmwareV2I_codes",       "FirmwareV2I_codes.bin" },
	// { "FirmwareV2I_segments",    "FirmwareV2I_segments.bin" },
	// { "FirmwareV2I_partitions",  "FirmwareV2I_partitions.bin" },
	// { "FirmwareV2I_scriptSets",  "FirmwareV2I_scriptSets.bin" },
	// { "FirmwareV2I_scripts",     "FirmwareV2I_scripts.bin" },
	// /* IT913x V2W */
	// { "FirmwareV2W_codes",       "FirmwareV2W_codes.bin" },
	// { "FirmwareV2W_segments",    "FirmwareV2W_segments.bin" },
	// { "FirmwareV2W_partitions",  "FirmwareV2W_partitions.bin" },
	// { "FirmwareV2W_scriptSets",  "FirmwareV2W_scriptSets.bin" },
	// { "FirmwareV2W_scripts",     "FirmwareV2W_scripts.bin" },
	// /* MxL HYDRA */
	// { "mxl_hydra_firmware_rawData", "mxl_hydra_firmware.bin" },
	// /* MxL Eagle */
	// { "mxl_eagle_firmware_rawData", "mxl_eagle_firmware.bin" },
};

#define FW_TABLE_SIZE (sizeof(fw_table) / sizeof(fw_table[0]))

/*
 * Convert a virtual address to a file offset using PT_LOAD program headers.
 * Returns -1 if the VA is not mapped (e.g. BSS beyond file extent).
 */
static off_t va_to_offset(const unsigned char *elf, Elf64_Addr va,
			   Elf64_Xword size, int *is_bss)
{
	const Elf64_Ehdr *ehdr = (const Elf64_Ehdr *)elf;
	const Elf64_Phdr *phdr = (const Elf64_Phdr *)(elf + ehdr->e_phoff);

	*is_bss = 0;

	for (int i = 0; i < ehdr->e_phnum; i++) {
		if (phdr[i].p_type != PT_LOAD)
			continue;
		if (va >= phdr[i].p_vaddr &&
		    va + size <= phdr[i].p_vaddr + phdr[i].p_memsz) {
			Elf64_Off off_in_seg = va - phdr[i].p_vaddr;
			if (off_in_seg + size > phdr[i].p_filesz) {
				/* Data falls beyond file-backed region → BSS */
				*is_bss = 1;
				return -1;
			}
			return phdr[i].p_offset + off_in_seg;
		}
	}
	return -1;
}

static int write_file(const char *name, const void *data, size_t size)
{
	FILE *f = fopen(name, "wb");
	if (!f) {
		perror(name);
		return -1;
	}
	if (fwrite(data, 1, size, f) != size) {
		perror(name);
		fclose(f);
		return -1;
	}
	fclose(f);
	printf("  %s (%zu bytes)\n", name, size);
	return 0;
}

static int write_zeroed(const char *name, size_t size)
{
	void *buf = calloc(1, size);
	if (!buf) {
		perror("calloc");
		return -1;
	}
	int ret = write_file(name, buf, size);
	free(buf);
	return ret;
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <liba3_phy_sony.so>\n", argv[0]);
		return 1;
	}

	int fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		perror(argv[1]);
		return 1;
	}

	struct stat st;
	if (fstat(fd, &st) < 0) {
		perror("fstat");
		close(fd);
		return 1;
	}

	unsigned char *elf = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	close(fd);
	if (elf == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	/* Validate ELF64 */
	const Elf64_Ehdr *ehdr = (const Elf64_Ehdr *)elf;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0 ||
	    ehdr->e_ident[EI_CLASS] != ELFCLASS64) {
		fprintf(stderr, "Not a valid ELF64 file\n");
		munmap(elf, st.st_size);
		return 1;
	}

	/* Find symbol table and string table.
	 * Prefer .symtab, fall back to .dynsym. */
	const Elf64_Shdr *shdr = (const Elf64_Shdr *)(elf + ehdr->e_shoff);
	const char *shstrtab = (const char *)(elf + shdr[ehdr->e_shstrndx].sh_offset);

	const Elf64_Shdr *symtab_sh = NULL;
	const Elf64_Shdr *dynsym_sh = NULL;

	for (int i = 0; i < ehdr->e_shnum; i++) {
		const char *name = shstrtab + shdr[i].sh_name;
		if (shdr[i].sh_type == SHT_SYMTAB && strcmp(name, ".symtab") == 0)
			symtab_sh = &shdr[i];
		else if (shdr[i].sh_type == SHT_DYNSYM && strcmp(name, ".dynsym") == 0)
			dynsym_sh = &shdr[i];
	}

	const Elf64_Shdr *use_sh = symtab_sh ? symtab_sh : dynsym_sh;
	if (!use_sh) {
		fprintf(stderr, "No symbol table found\n");
		munmap(elf, st.st_size);
		return 1;
	}

	const Elf64_Sym *symtab = (const Elf64_Sym *)(elf + use_sh->sh_offset);
	size_t nsyms = use_sh->sh_size / use_sh->sh_entsize;
	const char *strtab = (const char *)(elf + shdr[use_sh->sh_link].sh_offset);

	printf("Extracting firmware from %s (%s, %zu symbols)\n",
	       argv[1], symtab_sh ? ".symtab" : ".dynsym", nsyms);

	int extracted = 0;

	for (size_t t = 0; t < FW_TABLE_SIZE; t++) {
		for (size_t s = 0; s < nsyms; s++) {
			const char *name = strtab + symtab[s].st_name;
			if (strcmp(name, fw_table[t].sym_name) != 0)
				continue;

			Elf64_Addr va = symtab[s].st_value;
			Elf64_Xword size = symtab[s].st_size;
			if (size == 0) {
				fprintf(stderr, "  %s: symbol has zero size, skipping\n",
					fw_table[t].out_name);
				break;
			}

			int is_bss;
			off_t off = va_to_offset(elf, va, size, &is_bss);

			if (is_bss) {
				write_zeroed(fw_table[t].out_name, size);
			} else if (off >= 0) {
				write_file(fw_table[t].out_name, elf + off, size);
			} else {
				fprintf(stderr, "  %s: could not resolve VA 0x%lx\n",
					fw_table[t].out_name, (unsigned long)va);
				break;
			}
			extracted++;
			break;
		}
	}

	printf("Extracted %d files\n", extracted);
	munmap(elf, st.st_size);
	return 0;
}
