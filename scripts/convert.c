
#include <stdio.h>
#include <stdlib.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

int main(int argc, char *argv[])
{
	char         *in_files, *out_files;
	unsigned char rbuffer[4], wbuffer[4];
	long          filesize, pos, remain;
	size_t          ret;
	FILE         *in, *out;

	if (argc != 3) {
		printf("Input and output file are to be specified\n");
		exit(1);
	}

	in_files  = argv[1];
	out_files = argv[2];

	printf("convert file: %s >> %s\n", in_files, out_files);

	in  = fopen(in_files, "rb");
	out = fopen(out_files, "wb");
	if (in == NULL || out == NULL) { /* open for write */
		printf("Cannot open an input and an output file.\n");
		exit(0);
	}

	ret = fseek(in, 0, SEEK_END);
	if (ret) {
		return EXIT_FAILURE;
	}

	filesize = ftell(in);
	if (filesize <= 0) {
		return EXIT_FAILURE;
	}
	printf("filesize: %ld\n", filesize);

	ret = fseek(in, 0, SEEK_SET);
	if (ret) {
		return EXIT_FAILURE;
	}

	for (pos = 0; pos < filesize / 4; pos++) {
		ret = fread(rbuffer, ARRAY_SIZE(rbuffer), sizeof(*rbuffer), in);
		if (ret != sizeof(*rbuffer)) {
			fprintf(stderr, "fread() failed: %zu\n", ret);
			exit(EXIT_FAILURE);
		}

		wbuffer[0] = rbuffer[3];
		wbuffer[1] = rbuffer[2];
		wbuffer[2] = rbuffer[1];
		wbuffer[3] = rbuffer[0];

		ret = fwrite(wbuffer, ARRAY_SIZE(wbuffer), sizeof(*wbuffer), out);
		if (ret != sizeof(*wbuffer)) {
			fprintf(stderr, "fwrite() failed: %zu\n", ret);
			exit(EXIT_FAILURE);
		}
	}

	if (filesize % 4) {
		printf("file not align to 4byte\n");
	}

	fclose(in);
	fclose(out);
	exit(EXIT_SUCCESS);
}
