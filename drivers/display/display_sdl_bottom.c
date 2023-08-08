/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2023 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <SDL.h>
#include "nsi_tracing.h"

int sdl_display_init_bottom(uint16_t height, uint16_t width,
			    void **window, void **renderer, void **texture)
{
	*window = SDL_CreateWindow("Zephyr Display", SDL_WINDOWPOS_UNDEFINED,
				   SDL_WINDOWPOS_UNDEFINED, width,
				   height, SDL_WINDOW_SHOWN);
	if (*window == NULL) {
		nsi_print_warning("Failed to create SDL window: %s", SDL_GetError());
		return -1;
	}

	*renderer = SDL_CreateRenderer(*window, -1, SDL_RENDERER_ACCELERATED);
	if (*renderer == NULL) {
		nsi_print_warning("Failed to create SDL renderer: %s",
				SDL_GetError());
		return -1;
	}

	*texture = SDL_CreateTexture(*renderer, SDL_PIXELFORMAT_ARGB8888,
				     SDL_TEXTUREACCESS_STATIC, width, height);
	if (*texture == NULL) {
		nsi_print_warning("Failed to create SDL texture: %s", SDL_GetError());
		return -1;
	}

	SDL_SetRenderDrawColor(*renderer, 0, 0, 0, 0xFF);
	SDL_RenderClear(*renderer);
	SDL_RenderPresent(*renderer);

	return 0;
}

void sdl_display_write_bottom(const uint16_t height, const uint16_t width,
			      const uint16_t x, const uint16_t y,
			      void *renderer, void *texture,
			      uint8_t *buf, bool display_on)
{
	SDL_Rect rect;

	rect.x = x;
	rect.y = y;
	rect.w = width;
	rect.h = height;

	SDL_UpdateTexture(texture, &rect, buf, 4 * rect.w);

	if (display_on) {
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, texture, NULL, NULL);
		SDL_RenderPresent(renderer);
	}
}

int sdl_display_read_bottom(const uint16_t height, const uint16_t width,
			    const uint16_t x, const uint16_t y,
			    void *renderer, void *buf, uint16_t pitch)
{
	SDL_Rect rect;

	rect.x = x;
	rect.y = y;
	rect.w = width;
	rect.h = height;

	return SDL_RenderReadPixels(renderer, &rect, 0, buf, pitch * 4U);
}

void sdl_display_blanking_off_bottom(void *renderer, void *texture)
{
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

void sdl_display_blanking_on_bottom(void *renderer)
{
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);
}

void sdl_display_cleanup_bottom(void **window, void **renderer, void **texture)
{
	if (*texture != NULL) {
		SDL_DestroyTexture(*texture);
		*texture = NULL;
	}

	if (*renderer != NULL) {
		SDL_DestroyRenderer(*renderer);
		*renderer = NULL;
	}

	if (*window != NULL) {
		SDL_DestroyWindow(*window);
		*window = NULL;
	}
}
