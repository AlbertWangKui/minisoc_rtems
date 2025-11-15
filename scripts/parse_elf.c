#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <libelf.h>
#include <gelf.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <wchar.h>
#include <unistd.h>
#include <err.h>

#define errx(code, fmt, args...) \
	{ \
		fprintf(stderr, #code ": error at line %d in function %s:", \
			__LINE__, __FUNCTION__); \
		fprintf(stderr, fmt "\n", ##args); \
		exit(1); \
	}

int edit_elf_file(const char *path)
{
	int       fd;
	Elf      *elf;
	Elf_Scn  *scn;
	GElf_Shdr shdr;
	Elf_Data *data;
	size_t    n, shstrndx, sz;
	char     *name;

	if (elf_version(EV_CURRENT) == EV_NONE)
		return 1;

	fd = open(path, O_RDWR, 0); //打开elf文件
	if (fd < 0) {
		printf("%s can not open\n", path);
		return -1;
	}

	elf = elf_begin(fd, ELF_C_RDWR, NULL);
	if (!elf) {
		printf("%s can not get elf desc\n", path);
		return -1;
	}

	if (elf_kind(elf) != ELF_K_ELF)
		errx(EXIT_FAILURE, "%s is not an ELF object.", path);

	/* 获取section个数 */
	if (elf_getshdrstrndx(elf, &shstrndx) != 0)
		errx(EXIT_FAILURE, "elf_getshdrstrndx() failed.");

	scn = NULL;
	while ((scn = elf_nextscn(elf, scn)) != NULL) {
		/* 获取 section hdr */
		if (gelf_getshdr(scn, &shdr) != &shdr)
			errx(EXIT_FAILURE, "getshdr () failed: %s.", elf_errmsg(-1));

		/* 获取 shdr.sh_name 指针 */
		if ((name = elf_strptr(elf, shstrndx, shdr.sh_name)) == NULL)
			errx(EXIT_FAILURE, "elf_strptr () failed: %s.", elf_errmsg(-1));

		//(void)printf("Section % -4.4jd%s\n", (uintmax_t)elf_ndxscn(scn), name);

		if (strcmp(name, ".text") == 0 || strcmp(name, ".vector") == 0) {
			printf("    find section: %s\n", name);

			data = NULL;
			n    = 0;
			if ((data = elf_getdata(scn, data)) != NULL) {
				//printf("size: 0x%lx\n", data->d_size);

				char *p = (char *)data->d_buf;
				char  buf[4];

				int count = data->d_size / 4;
				int extra = data->d_size % 4;
				int i;

				if(extra)
					errx(EXIT_FAILURE, "section size not align");

				for (i = 0; i < count; i++) {
					memcpy(buf, p, 4);

					p[0] = buf[2];
					p[1] = buf[3];
					p[2] = buf[0];
					p[3] = buf[1];

					p = p + 4;
				}

				elf_flagdata(data, ELF_C_SET, ELF_F_DIRTY);
			}
		}
	}

	if (elf_update(elf, ELF_C_WRITE) < 0)
		errx(EXIT_FAILURE, "elf_update() failed: %s.", elf_errmsg(-1));

	(void)elf_end(elf);
	(void)close(fd);
}

int file_cp(char *src_path, char *dest_path)
{
#ifndef BUF_SIZE /* Allow "cc -D" to override definition */
#define BUF_SIZE 1024
#endif
	int     fd_in = -1, fd_out = -1;
	char    buf[BUF_SIZE];
	ssize_t numread;

	if ((fd_in = open(src_path, O_RDONLY, 0)) < 0)
		err(EXIT_FAILURE, "open \"%s\" failed", src_path);

	if ((fd_out = open(dest_path, O_RDWR | O_CREAT, 0777)) < 0)
		err(EXIT_FAILURE, "open \"%s\" failed", dest_path);

	/* Transfer data until we encounter end of input or an error */
	while ((numread = read(fd_in, buf, BUF_SIZE)) > 0) {
		if (write(fd_out, buf, numread) != numread)
			err(EXIT_FAILURE, "write() returned error or partial write occurred");
	}

	if (numread == -1)
		err(EXIT_FAILURE, "read");

	close(fd_in);
	close(fd_out);
}

int main(int argc, char *argv[])
{
	if (argc != 3)
		errx(EX_USAG, "\nUSAGE: %s in-file-name out-file-name", argv[0]);

	file_cp(argv[1], argv[2]);
	edit_elf_file(argv[2]);

	return 0;
}
