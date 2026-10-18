#include "protocol.h"
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
bool protocol_is_v2_handshake(const char *line) {
    return strcmp(line, "BASE_CONTROL_V2")==0;
}
bool parse_velocity(const char *line, float *left, float *right) {
    int end = 0;
    if (sscanf(line, "v %f %f %n", left, right, &end)!=2 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && isfinite(*left) && isfinite(*right) && fabsf(*left)<=100 && fabsf(*right)<=100;
}
bool parse_duty(const char *line, float *left, float *right) {
    int end = 0;
    if (sscanf(line, "d %f %f %n", left, right, &end)!=2 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && isfinite(*left) && isfinite(*right) &&
           *left>=0 && *right>=0 && *left<=.65f && *right<=.65f;
}
bool parse_loaded(const char *line, float *left, float *right) {
    int end = 0;
    if (sscanf(line, "b %f %f %n", left, right, &end)!=2 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && isfinite(*left) && isfinite(*right) &&
           *left>=150 && *right>=150 && *left<=200 && *right<=200;
}
bool parse_arc(const char *line, float *delta) {
    int end = 0;
    if (sscanf(line, "a %f %n", delta, &end)!=1 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && isfinite(*delta) && fabsf(*delta)<=50;
}
bool parse_sync(const char *line, float *direction) {
    int end = 0;
    if (sscanf(line, "y %f %n", direction, &end)!=1 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && (*direction==1 || *direction==-1);
}
bool parse_turn(const char *line, float *direction) {
    int end = 0;
    if (sscanf(line, "t %f %n", direction, &end)!=1 || end==0) return false;
    while (line[end] && isspace((unsigned char)line[end])) ++end;
    return !line[end] && (*direction==1 || *direction==-1);
}
