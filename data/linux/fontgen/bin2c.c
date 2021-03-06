#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv) {
	FILE *f1, *f2;
	unsigned char buf[12];
	unsigned int count, i, size = 0;

	if (argc != 4) {
		fprintf(stderr, "Syntax: bin2c <infile> <outfile> <variable name>\n");
		return -1;
	}

	f1 = fopen(argv[1], "rb");

	if (!f1) {
		fprintf(stderr, "Error opening %s\n", argv[1]);
		return -1;
	}

	f2 = fopen(argv[2], "w");
	if (!f2) {
		fprintf(stderr, "Error opening %s\n", argv[2]);
		return -1;
	}

	fprintf(f2, "/* Autogenerated by bin2c */\n\n");
	fprintf(f2, "uint8_t %s[] = {\n", argv[3]);

	while ((count = fread(buf, 1, sizeof(buf), f1)) > 0) {
		fprintf(f2, "\t");
		for (i=0; i<count; i++) {
			fprintf(f2, "0x%.2x, ", (unsigned int) buf[i]);
			size++;
		}
		fprintf(f2, "\n");
	}

	fprintf(f2, "};\n\n");
	fprintf(f2, "uint32_t %s_size = %i;\n", argv[3], size);

	fclose(f1);
	fclose(f2);
	
	return 0;
}
