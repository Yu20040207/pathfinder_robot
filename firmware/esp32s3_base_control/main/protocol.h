#pragma once
#include <stdbool.h>
bool parse_velocity(const char *line, float *left, float *right);
bool parse_duty(const char *line, float *left, float *right);
bool parse_loaded(const char *line, float *left, float *right);
bool parse_arc(const char *line, float *delta);
bool parse_sync(const char *line, float *direction);
bool parse_turn(const char *line, float *direction);
bool protocol_is_v2_handshake(const char *line);
