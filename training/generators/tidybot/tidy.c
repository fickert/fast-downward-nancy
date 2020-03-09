#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

typedef int32_t i32;
typedef int64_t i64;
typedef int8_t i8;
typedef uint8_t u8;

typedef struct Point
{
	i32 x;
	i32 y;
} Point;

typedef struct Box
{
	i32 xmin, xmax, ymin, ymax;
} Box;

typedef struct Table
{
	Box b;
} Table;

typedef struct Cupboard
{
	Box b;
	i8 orientation;
} Cupboard;

typedef struct Surfaces
{
	Table *tables;
	Cupboard *cupboards;
	i32 ntbls;
	i32 ncpbs;
} Surfaces;

typedef struct PossibleObjectLocation
{
	Point *p;
	i32 n;
} PossibleObjectLocation;

typedef struct PossibleLocations
{
	Point *tbl_options;
	i32 ntbls;
	Point *cpb_options;
	i32 ncpbs;
	i32 cap;

	PossibleObjectLocation *by_cupboard;
} PossibleLocations;

typedef struct ObjectLocations
{
	struct {
		Point p[3];
		bool has_table;
	} *locs;
	i32 n;
} ObjectLocations;

i32 clamp(i32 n, i32 max)
{
	return n > max ? max : n;
}

i32 rand_between(i32 min, i32 max)
{
	return rand() % (max + 1 - min) + min;
}

Box sample_box(i32 ws, i32 min, i32 max)
{
	i32 xsize = rand_between(min, max);
	i32 ysize = rand_between(min, max);
	i32 minx  = 1 + rand() % (ws - xsize - 2);
	i32 miny  = 1 + rand() % (ws - ysize - 2);
	return (Box){.xmin=minx, .ymin=miny, .xmax=clamp(minx+xsize-1,ws), .ymax=clamp(miny+ysize-1,ws)};
}

bool inside(Box const *b, i32 i, i32 j)
{
	return b->xmin < i && b->ymin < j && i < b->xmax && j < b->ymax;
}

bool too_close(Box const *b, i32 x, i32 y)
{
	return b->xmin - 1 <= x && b->ymin - 1 <= y && x <= b->xmax + 1 && y <= b->ymax +1;
}

bool collide_one(Box const *b1, Box const*b2)
{
	return too_close(b2, b1->xmin, b1->ymin) || too_close(b2, b1->xmin, b1->ymax) || too_close(b2, b1->xmax, b1->ymin) || too_close(b2, b1->xmax, b1->ymax);
}

bool collide(Surfaces const *surfaces, Box const *b)
{
	for (i32 i = 0; i < surfaces->ntbls; ++i) {
		if (collide_one(&surfaces->tables[i].b, b) ||
		    collide_one(b, &surfaces->tables[i].b))
			return true;
	}
	for (i32 i = 0; i < surfaces->ncpbs; ++i) {
		if (collide_one(&surfaces->cupboards[i].b, b) ||
		    collide_one(b, &surfaces->cupboards[i].b))
			return true;
	}
	return false;
}

void try_place_surface(Surfaces *surfaces, i32 ws, i32 min, i32 max, u8 kind)
{
	Box b;
	for (i32 i = 0; i < 100; ++i) {
		b = sample_box(ws, min, max);
		if (!collide(surfaces, &b)) {
			if (kind == 't') {
				surfaces->tables[surfaces->ntbls] = (Table){.b=b};
				++surfaces->ntbls;
			} else {
				surfaces->cupboards[surfaces->ncpbs] = (Cupboard){.b=b, .orientation=kind};
				++surfaces->ncpbs;
			}
			break;
		}
	}
}

void generate_cupboards(i32 ws, i32 count, i32 size, Surfaces *surfaces)
{
	for (i32 i = 0; i < count; ++i) {
		try_place_surface(surfaces, ws, size, size, rand() % 4);
	}
}

void generate_tables(i32 ws, i32 count, i32 mins, i32 maxs, Surfaces *surfaces)
{
	for (i32 i = 0; i < count; ++i) {
		try_place_surface(surfaces, ws, mins, maxs, 't');
	}
}

bool location_possible(Box const *b, i32 i, i32 j, bool table)
{
	return inside(b, i, j) ^ table;
}

void place_objects(Surfaces const *surfaces, PossibleLocations *locs, ObjectLocations *ol)
{
	fprintf(stderr, "placing objects\n");
	i32 max = surfaces->ntbls;
	if (surfaces->ncpbs > max)
		max = surfaces->ncpbs;
	locs->tbl_options = calloc(sizeof(locs->tbl_options[0]), max);
	locs->ntbls = 0;
	locs->cpb_options = calloc(sizeof(locs->cpb_options[0]), max);
	locs->ncpbs = 0;
	locs->cap = max;
	// this is always enough here
	locs->by_cupboard = calloc(sizeof(locs->by_cupboard[0]), surfaces->ncpbs);

	// compute possible locations for tables
	for (i32 n = 0; n < surfaces->ntbls; ++n) {
		for (i32 i = surfaces->tables[n].b.xmin; i < surfaces->tables[n].b.xmax; ++i) {
			for (i32 j = surfaces->tables[n].b.ymin; j < surfaces->tables[n].b.ymax; ++j) {
				if (location_possible(&surfaces->tables[n].b, i, j, true)) {
					if (locs->ntbls >= locs->cap) {
						locs->cap *= 2;
						locs->tbl_options = realloc(locs->tbl_options, sizeof(locs->tbl_options[0]) * locs->cap);
						free(locs->cpb_options);
						locs->cpb_options = calloc(sizeof(locs->cpb_options[0]), locs->cap);
					}
					locs->tbl_options[locs->ntbls].x = i;
					locs->tbl_options[locs->ntbls].y = j;
					++locs->ntbls;
				}
			}
		}
	}

	// compute possible locations for cupboards
	for (i32 n = 0; n < surfaces->ncpbs; ++n) {
		fprintf(stderr, "mallocing %d for by cupboards array\n", (surfaces->cupboards[n].b.xmax - surfaces->cupboards[n].b.xmin) * (surfaces->cupboards[n].b.ymax - surfaces->cupboards[n].b.xmax));
		locs->by_cupboard[n].p = calloc(sizeof(locs->by_cupboard[n].p[0]),
						(surfaces->cupboards[n].b.xmax + 1 - surfaces->cupboards[n].b.xmin)
						* (surfaces->cupboards[n].b.ymax + 1 - surfaces->cupboards[n].b.xmax));
		locs->by_cupboard[n].n = 0;
		for (i32 i = surfaces->cupboards[n].b.xmin; i < surfaces->cupboards[n].b.xmax; ++i) {
			for (i32 j = surfaces->cupboards[n].b.xmin; j < surfaces->cupboards[n].b.ymax; ++j) {
				if (location_possible(&surfaces->cupboards[n].b, i, j, false)) {
					if (locs->ncpbs >= locs->cap) {
						locs->cap *= 2;
						locs->cpb_options = realloc(locs->cpb_options, sizeof(locs->cpb_options[0]) * locs->cap);
					}
					locs->cpb_options[locs->ncpbs].x = i;
					locs->cpb_options[locs->ncpbs].y = j;
					++locs->ncpbs;

					i32 *k = &locs->by_cupboard[n].n;
					locs->by_cupboard[n].p[*k].x = i;
					locs->by_cupboard[n].p[*k].y = j;
					*k = *k + 1;
				}
			}
		}
	}

	// compute 'init locations'.  the init locations are
	// number-of-cupboards-many distinct and randomly chosen
	// locations (out of all locations, not just the cupboard
	// locations).
	i32 ntbl_locs = locs->ntbls;
	i32 ncpb_locs = locs->ncpbs;
	i32 ntotal = ntbl_locs + ncpb_locs;
	Point *init_locs = malloc(sizeof(init_locs[0]) * locs->ncpbs);
	Point tmp;
	i32 used_locs = locs->ncpbs;
	for (i32 i = 0; i < locs->ncpbs; ++i) {
		if (ntotal == 0) {
			used_locs = i;
			break;
		}
		i32 sidx = rand() % ntotal;
		if (sidx < locs->ntbls) {
			--ntbl_locs;
			--ntotal;
			init_locs[i] = locs->tbl_options[sidx];
			tmp = locs->tbl_options[sidx];
			locs->tbl_options[sidx] = locs->tbl_options[ntbl_locs];
			locs->tbl_options[ntbl_locs] = tmp;
		} else {
			sidx -= locs->ntbls;
			--ncpb_locs;
			--ntotal;
			init_locs[i] = locs->cpb_options[sidx];
			tmp = locs->cpb_options[sidx];
			locs->cpb_options[sidx] = locs->cpb_options[ncpb_locs];
			locs->cpb_options[ncpb_locs] = tmp;
		}
	}

	// compute the 'object locations'.  in the original clojure
	// source, the object locations are a a pair x y, where x is
	// one of the init locations and y is a list containing a
	// cupboard location and 0 or 1 table locations.  Here, I
	// flatten that to a contiguous array, of 2 or 3 points.
	ol->n = used_locs;
	ol->locs = malloc(sizeof(ol->locs[0]) * used_locs);
	for (i32 i = 0; i < used_locs; ++i) {
		ol->locs[i].p[0] = init_locs[i];
		ol->locs[i].p[1] = locs->cpb_options[i];
		if (locs->ntbls) {
			ol->locs[i].has_table = rand() & 1;
			if (ol->locs[i].has_table) {
				i32 tidx = rand() % locs->ntbls;
				ol->locs[i].p[2] = locs->tbl_options[tidx];
			}
		} else {
			ol->locs[i].has_table = false;
		}
	}

	free(init_locs);
}

void make_instance(char *problemname, char *filename, i32 w, i32 h,
		   char *robotname, i32 robotx, i32 roboty,
		   char *cartname, i32 cartx, i32 carty,
		   Surfaces const *surfaces,
		   PossibleLocations const *locs,
		   ObjectLocations const *ol,
		   i32 gripper_radius)
{
	printf("(define\n(problem %s)\n(:domain TIDYBOT)\n\n(:objects\npr2 - robot\ncart - cart\n", problemname);
	for (i32 i = 0; i < ol->n; ++i) {
		printf("object%d - object\n", i);
	}
	for (i32 i = 0; i < w; ++i) {
		printf("x%d - xc\n", i);
	}
	for (i32 i = 0; i < h; ++i) {
		printf("y%d - yc\n", i);
	}
	for (i32 i = -gripper_radius; i <= gripper_radius; ++i) {
		printf("xrel%d - xrel\nyrel%d - yrel\n", i);
	}
	printf("\n)\n\n(:init\n");
	for (i32 i = 0; i < w - 1; ++i) {
		printf("(leftof x%d X%d)\n", i, i+1);
	}
	for (i32 i = 0; i < h - 1; ++i) {
		printf("(above y%d y%d)\n", i, i +1);
	}
	for (i32 i = -gripper_radius; i < gripper_radius; ++i) {
		printf("(leftof-rel xrel%d xrel%d)\n", i, i+1);
	}
	for (i32 i = -gripper_radius; i < gripper_radius; ++i) {
		printf("(above-rel yrel%d yrel%d)\n", i, i+1);
	}

	for (i32 i = 0; i < w; ++i) {
		for (i32 j = -gripper_radius; j <= gripper_radius; ++j) {
			i32 sum = i + j;
			if (0 <= sum && sum < w) {
				printf("(sum-x x%d xrel%d x%d)\n", i, j, sum);
			}
		}
	}
	for (i32 i = 0; i < h; ++i) {
		for (i32 j = -gripper_radius; j <= gripper_radius; ++j) {
			i32 sum = i + j;
			if (0 <= sum && sum < h) {
				printf("(sum-y y%d yrel%d y%d)\n", i, j, sum);
			}
		}
	}
	printf("(zerox-rel xrel0)\n(zeroy-rel yrel0)\n");

	for (i32 i = 0; i < ol->n; ++i) {
		printf("(object-goal object%d x%d y%d)\n", i, ol->locs[i].p[1].x, ol->locs[i].p[1].y);
	}

	printf("\n(parked %s)\n(not-pushing %s)\n(base-pos %s x%d y%d)\n(base-obstacle x%d y%d)\n", robotname, robotname, robotname, robotx, roboty, robotx, roboty);

	for (i32 i = 0; i < surfaces->ntbls; ++i) {
		Box const *b = &surfaces->tables[i].b;
		for (i32 x = b->xmin; x <= b->xmax; ++x) {
			for (i32 y = b->ymin; y <= b->ymax; ++y) {
				printf("(base-obstacle x%d y%d)(surface x%d y%d)\n", x, y, x, y);
			}
		}
	}

	for (i32 i = 0; i < surfaces->ncpbs; ++i) {
		Box const *b = &surfaces->cupboards[i].b;
		for (i32 x = b->xmin; x <= b->xmax; ++x) {
			for (i32 y = b->ymin; y <= b->ymax; ++y) {
				printf("(base-obstacle x%d y%d)(gripper-obstacle x%d y%d)\n", x, y, x, y);
			}
		}
		for (i32 j = 0; j < locs->by_cupboard[i].n; ++j) {
			printf("(surface x%d y%d)\n", locs->by_cupboard[i].p[j].x, locs->by_cupboard[i].p[j].y);
		}
	}

	printf("\n(cart-pos %s x%d y%d)\n(not-pushed %s)\n(base-obstacle x%d y%d)\n\n", cartname, cartx, carty, cartname, cartx, carty);

	for (i32 i = 0; i < ol->n; ++i) {
		printf("(object-pos object%d x%d y%d)\n(gripper-obstacle x%d y%d)\n", i, ol->locs[i].p[0].x, ol->locs[i].p[0].y, ol->locs[i].p[0].x, ol->locs[i].p[0].y);
	}

	printf("\n(gripper-empty %s)\n(gripper-rel %s xrel%d yrel%d))\n\n(:goal\n(and\n", robotname, robotname, robotx, roboty);

	for (i32 i = 0; i < ol->n; ++i) {
		printf("(object-done object%d)\n", i);
	}
	printf(")))\n");
}

void generate_world(i32 ws, i32 nt, i32 nc, i32 mins, i32 maxs, i32 cs)
{
	if (ws <= 0 || nt < 0 || nc < 0 || mins <= 0 || maxs <= 0 || mins > maxs || cs <= 0) {
		fprintf(stderr, "sizes should be greater than zero, the number of objects shouldn't be less than zero, and max size should be at least min size\n");
		return;
	}
	Surfaces surfaces;
	PossibleLocations locs;
	ObjectLocations ol;

	surfaces.tables = nt > 0 ? malloc(sizeof(surfaces.tables[0]) * nt) : NULL;
	surfaces.ntbls = 0;
	surfaces.cupboards = nc > 0 ? malloc(sizeof(surfaces.cupboards[0]) * nc) : NULL;
	surfaces.ncpbs = 0;
	generate_cupboards(ws, nc, cs, &surfaces);
	generate_tables(ws, nt, mins, maxs, &surfaces);
	place_objects(&surfaces, &locs, &ol);

	make_instance("test", "filename", ws, ws, "pr2", 0, 0, "cart", 0, 1, &surfaces, &locs, &ol, 1);

	for (i32 i = 0; i < surfaces.ncpbs; ++i) {
		free(locs.by_cupboard[i].p);
	}
	free(locs.by_cupboard);
	free(surfaces.tables);
	free(surfaces.cupboards);
	free(locs.tbl_options);
	free(locs.cpb_options);
	free(ol.locs);
}

i32 main(i32 argc, u8 **argv)
{
	if (argc != 7 && argc != 8) {
		fprintf(stderr, "Usage:\ntidy world_size n_tables n_cupboards min_surface_size max_surface_size cupboard_size [random_seed]\n");
		return 1;
	}

	i32 ws = atoi(argv[1]);
	i32 nt = atoi(argv[2]);
	i32 nc = atoi(argv[3]);
	i32 mins = atoi(argv[4]);
	i32 maxs = atoi(argv[5]);
	i32 cs = atoi(argv[6]);
	i32 seed = argc == 8 ? atoi(argv[7]) : 1;
	srand(seed);
	generate_world(ws, nt, nc, mins, maxs, cs);
}
