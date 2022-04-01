from http.server import ThreadingHTTPServer
import os
from turtle import back
import cv2
import numpy as np
import pygame

background = cv2.imread('./C3_map.png')

grass = cv2.inRange(background, (70, 160, 100), (85, 175, 120))
grass_edge = cv2.Canny(grass, threshold1=100, threshold2=200)
yellow_lines = cv2.inRange(background, (0, 225, 225), (5, 255, 255))
white_lines = cv2.inRange(background, (250, 250, 250), (255, 255, 255))
cv2.imwrite('./yellow_lines.png', yellow_lines)
cv2.imwrite('./white_lines.png', white_lines)

WIDTH, HEIGHT = background.shape[:2]
WIN = pygame.display.set_mode((int(WIDTH/2), int(HEIGHT/2)))
pygame.display.set_caption("Car Test")

BACKGROUND = pygame.transform.scale(pygame.image.load('./C3_map.png'), (int(WIDTH/2), int(HEIGHT/2)))



# Car Initiallization
CAR_WIDTH, CAR_HEIGHT = 60, 60
CAR_IMAGE = pygame.image.load(os.path.join('car.png'))
CAR_DIRECTION = 90
CAR = pygame.transform.rotate(pygame.transform.scale(
    CAR_IMAGE, (CAR_WIDTH//2, CAR_HEIGHT)), CAR_DIRECTION+90)


# Speed Information
VEL = 2

# Event Information
OFF_ROAD = pygame.USEREVENT + 1
CROSS_CENTER = pygame.USEREVENT + 4
CROSS_STOP = pygame.USEREVENT + 3
LEAVE_SCREEN = pygame.USEREVENT + 4

def handle_movement(keys_pressed, car_rect):
    vel = 0
    net_direct = 0
    global CAR_DIRECTION
    global CAR
    if keys_pressed[pygame.K_LEFT]:  # LEFT
        net_direct += -5
    if keys_pressed[pygame.K_RIGHT]:  # RIGHT
        net_direct += 5
    if keys_pressed[pygame.K_UP]:  # UP
        vel += VEL
    if keys_pressed[pygame.K_DOWN]:  # DOWN
        vel += -VEL
    new_dir = CAR_DIRECTION + net_direct
    if new_dir > 360:
        new_dir += -360
    elif new_dir < 0:
        new_dir + 360
    CAR_DIRECTION = new_dir
    CAR = pygame.transform.rotate(pygame.transform.scale(
        CAR_IMAGE, (CAR_WIDTH, CAR_HEIGHT)),
        CAR_DIRECTION + 90)
    car_rect.x += vel * np.cos(CAR_DIRECTION*np.pi/180)
    car_rect.y += vel * np.sin(CAR_DIRECTION*np.pi/180)
    # Check Crashes
    
    return car_rect

def draw(car_rect):
    WIN.blit(BACKGROUND, (0,0))
    WIN.blit(CAR, (car_rect.x, car_rect.y))

    pygame.display.update()


def main():
    car_rect = pygame.Rect(200, 200, CAR_WIDTH, CAR_HEIGHT)

    clock = pygame.time.Clock()
    run = True
    while run:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                break
            #need all the intersect stuff

        keys_pressed = pygame.key.get_pressed()
        car_rect = handle_movement(keys_pressed, car_rect)

        draw(car_rect)



if __name__ == '__main__':
    main()