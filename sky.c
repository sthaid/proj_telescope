#include "common.h"

//
// defines
//

#define MAX_OBJECT 200000

//
// typedefs
//

typedef struct {
    char *proper;
    float ra;
    float dec;
} object_t;

//
// variables
//

object_t object[MAX_OBJECT];
int      max_object;

//
// prototypes
//

int read_sky_data(char * filename);

// -----------------  XXXX  -----------------------------------------------

int sky_init(void) 
{

    int ret;

    ret = read_sky_data("sky_data/hygdata_v3.csv");
    if (ret < 0) {
        return ret;
    }

    ret = read_sky_data("sky_data/more.csv");
    if (ret < 0) {
        return ret;
    }
    // open and parse sky_data/hygdata_v3.csv


    return 0;
}

int read_sky_data(char * filename)
{
    // csv file format:
    //   id,hip,hd,hr,gl,bf,proper,ra,dec,
    //      dist,pmra,pmdec,rv,mag,absmag,spect,ci,x,y,z,vx,vy,vz,rarad,decrad,pmrarad,pmdecrad,bayer,
    //      flam,con,comp,comp_primary,base,lum,var,var_min,var_max

    #define GET_FIELD(field) \
        do { \
            field = s; \
            s = strchr(s,','); \
            if (s == NULL) { \
                ERROR("filename=%s line=%d field=%s\n", filename, line, #field); \
                return -1; \
            } \
            *s = '\0'; \
            s++; \
        } while (0)

    FILE *fp;
    int line=1, num_added=0, len;
    char str[10000], *s;
    char *id, *hip, *hd, *hr, *gl, *bf, *proper, *ra, *dec;
    char *proper_val;
    float ra_val, dec_val;

    // open file
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        s=str;
        GET_FIELD(id);
        GET_FIELD(hip);
        GET_FIELD(hd);
        GET_FIELD(hr);
        GET_FIELD(gl);
        GET_FIELD(bf);
        GET_FIELD(proper);
        GET_FIELD(ra);
        GET_FIELD(dec);

        if (line == 1) {
            if (strcmp(proper, "proper") || strcmp(ra, "ra") || strcmp(dec, "dec")) {
                ERROR("csv file header line incorrect, proper='%s' ra=%s dec=%s\n", proper, ra, dec);
                return -1;
            }
            line++;
            continue;
        } 

        if (sscanf(ra, "%f", &ra_val) != 1) {
            ERROR("filename=%s line=%d invalid ra='%s'\n", filename, line, ra);
            return -1;
        }
        if (sscanf(dec, "%f", &dec_val) != 1) {
            ERROR("filename=%s line=%d invalid dec='%s'\n", filename, line, dec);
            return -1;
        }
        len = strlen(proper);
        if (len) {
            proper_val = malloc(len+1);
            strcpy(proper_val, proper);
        } else {
            proper_val = "";
        }

        object[max_object].proper = proper_val;
        object[max_object].ra     = ra_val;
        object[max_object].dec    = dec_val;
        max_object++;

        num_added++;
        line++;
    }

    // close file
    fclose(fp);

    // success
    INFO("added %d objects from %s\n", num_added, filename);
    return 0;
}









int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int tbd;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        rect_t * pane = &pane_cx->pane;
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_MOUSE_WHEEL:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_HOME:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_END:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGUP:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_UP_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_DOWN_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}


