import os
import numpy as np
import pygame

WIDTH, HEIGHT = 800, 800
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car Test")

COLOR_GRASS = (98, 152, 60)
COLOR_CENTER_LINE = (255, 255, 0)
COLOR_ROAD = (0,0,0)
COLOR_STOP_LINE = (255, 255, 255)


# Off the Road
TL_GRASS = pygame.Rect(0, 0, WIDTH//3+5, HEIGHT//3+5)
TR_GRASS = pygame.Rect(2*WIDTH//3-5, 0, WIDTH//3+10, HEIGHT//3+10)
BL_GRASS = pygame.Rect(0, 2*HEIGHT//3-5, WIDTH//3+10, HEIGHT//3+10)
BR_GRASS = pygame.Rect(2*WIDTH//3-5, 2*HEIGHT//3-5, WIDTH//3+10, HEIGHT//3+10)

# Roads
VERT_ROAD = pygame.Rect(WIDTH//3, 0, WIDTH//3, HEIGHT)
HORI_ROAD = pygame.Rect(0, HEIGHT//3, WIDTH, HEIGHT//3)

# Center Lines
TOP_CENTER_LINE = pygame.Rect(WIDTH//2 - 2, 0, 4, HEIGHT//3)
BOTTOM_CENTER_LINE = pygame.Rect(WIDTH//2 - 2, 2*HEIGHT//3, 4, HEIGHT//3)
LEFT_CENTER_LINE = pygame.Rect(0, HEIGHT//2 - 2, WIDTH//3, 4)
RIGHT_CENTER_LINE = pygame.Rect(2*WIDTH//3, HEIGHT//2 - 2, WIDTH//3, 4)

# Stopping Lines
TOP_STOP_LINE = pygame.Rect(WIDTH//3, HEIGHT//3 - 10, WIDTH//6, 10)
BOTTOM_STOP_LINE = pygame.Rect(WIDTH//2, 2*HEIGHT//3, WIDTH//6, 10)
LEFT_STOP_LINE = pygame.Rect(WIDTH//3-10, HEIGHT//2, 10, HEIGHT//6)
RIGHT_STOP_LINE = pygame.Rect(2*WIDTH//3, HEIGHT//3, 10, HEIGHT//6)

# Collection
GRASS_SPOTS = [TL_GRASS, TR_GRASS, BL_GRASS, BR_GRASS]
ROAD_SPOTS = [VERT_ROAD, HORI_ROAD]
CENTER_LINES = [TOP_CENTER_LINE, BOTTOM_CENTER_LINE, LEFT_CENTER_LINE, RIGHT_CENTER_LINE]
STOP_LINES = [TOP_STOP_LINE, BOTTOM_STOP_LINE, LEFT_STOP_LINE, RIGHT_STOP_LINE]

# Car Initiallization
CAR_WIDTH, CAR_HEIGHT = 60, 60
CAR_IMAGE = pygame.image.load(os.path.join('car.png'))
CAR_DIRECTION = [0, 5]
CAR = pygame.transform.rotate(pygame.transform.scale(
    CAR_IMAGE, (CAR_WIDTH//2, CAR_HEIGHT)),
    np.degrees(np.arctan2(CAR_DIRECTION[1], CAR_DIRECTION[0]))+90)


# Speed Information
VEL = 2

# Event Information
OFF_ROAD = pygame.USEREVENT + 1
CROSS_CENTER = pygame.USEREVENT + 4
CROSS_STOP = pygame.USEREVENT + 3
LEAVE_SCREEN = pygame.USEREVENT + 4

def handle_movement(keys_pressed, car_rect):
    new_direction = [0, 0]
    global CAR_DIRECTION
    global CAR
    if keys_pressed[pygame.K_LEFT]:  # LEFT
        new_direction[0] -= VEL
    if keys_pressed[pygame.K_RIGHT]:  # RIGHT
        new_direction[0] += VEL
    if keys_pressed[pygame.K_UP]:  # UP
        new_direction[1] -= VEL
    if keys_pressed[pygame.K_DOWN]:  # DOWN
        new_direction[1] += VEL
    if new_direction != [0, 0]:
        CAR_DIRECTION = new_direction
    CAR = pygame.transform.rotate(pygame.transform.scale(
        CAR_IMAGE, (CAR_WIDTH, CAR_HEIGHT)),
        np.degrees(np.arctan2(-CAR_DIRECTION[1], CAR_DIRECTION[0]))+90)
    car_rect.x += new_direction[0]
    car_rect.y += new_direction[1]
    # Check Crashes
    if car_rect.colliderect(GRASS_SPOTS[0]) or car_rect.colliderect(GRASS_SPOTS[1]) or car_rect.colliderect(GRASS_SPOTS[2]) or car_rect.colliderect(GRASS_SPOTS[3]):
        pygame.event.post(pygame.event.Event(OFF_ROAD))
        CAR_DIRECTION = [0, 5]
        CAR = pygame.transform.rotate(pygame.transform.scale(
            CAR_IMAGE, (CAR_WIDTH, CAR_HEIGHT)),
            np.degrees(np.arctan2(CAR_DIRECTION[1], CAR_DIRECTION[0]))+90)
        car_rect = pygame.Rect(WIDTH//2+30, HEIGHT - 60, CAR_WIDTH, CAR_HEIGHT)
        pygame.time.delay(500)
    if car_rect.x < 0 or car_rect.x > WIDTH or car_rect.y < 0 or car_rect.y > HEIGHT:
        pygame.event.post(pygame.event.Event(LEAVE_SCREEN))
        CAR_DIRECTION = [0, 5]
        CAR = pygame.transform.rotate(pygame.transform.scale(
            CAR_IMAGE, (CAR_WIDTH, CAR_HEIGHT)),
            np.degrees(np.arctan2(CAR_DIRECTION[1], CAR_DIRECTION[0]))+90)
        car_rect = pygame.Rect(WIDTH//2+30, HEIGHT - 60, CAR_WIDTH, CAR_HEIGHT)
        pygame.time.delay(500)
    if car_rect.colliderect(CENTER_LINES[0]) or car_rect.colliderect(CENTER_LINES[1]) or car_rect.colliderect(CENTER_LINES[2]) or car_rect.colliderect(CENTER_LINES[3]):
        pygame.event.post(pygame.event.Event(CROSS_CENTER))
    if car_rect.colliderect(STOP_LINES[0]) or car_rect.colliderect(STOP_LINES[1]) or car_rect.colliderect(STOP_LINES[2]) or car_rect.colliderect(STOP_LINES[3]):
        pygame.event.post(pygame.event.Event(CROSS_STOP))
    return car_rect

def draw(car_rect):
    for grass in GRASS_SPOTS:
        pygame.draw.rect(WIN, COLOR_GRASS, grass)
    for road in ROAD_SPOTS:
        pygame.draw.rect(WIN, COLOR_ROAD, road)
    for line in CENTER_LINES:
        pygame.draw.rect(WIN, COLOR_CENTER_LINE, line)
    for line in STOP_LINES:
        pygame.draw.rect(WIN, COLOR_STOP_LINE, line)
    WIN.blit(CAR, (car_rect.x, car_rect.y))

    pygame.display.update()


def main():
    car_rect = pygame.Rect(WIDTH//2+30, HEIGHT - 100, CAR_WIDTH, CAR_HEIGHT)

    clock = pygame.time.Clock()
    run = True
    while run:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                break

            if event.type == OFF_ROAD:
                print(f'Car fell off the road at ({car_rect.x}, {car_rect.y})...')

            if event.type == CROSS_CENTER:
                print(f'Car crossed the center line at ({car_rect.x}, {car_rect.y})...')

            if event.type == CROSS_STOP:
                print(f'Car crossed the stop line at ({car_rect.x}, {car_rect.y})...')

            if event.type == LEAVE_SCREEN:
                print(f'Car left the screen at ({car_rect.x}, {car_rect.y})...')

        keys_pressed = pygame.key.get_pressed()
        car_rect = handle_movement(keys_pressed, car_rect)

        draw(car_rect)



if __name__ == '__main__':
    main()