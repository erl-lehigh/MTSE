import os
import cv2
import numpy as np
import pygame

from map_info import map_start

image_file = './C3_map.png'
# image_file = './merge_map.png'
start_loc = map_start[image_file]
background = cv2.imread(image_file)

spawn_x = start_loc[0]
spawn_y = start_loc[1]

grass = cv2.inRange(background, (70, 160, 100), (85, 175, 120))
grass_edge = cv2.Canny(grass, threshold1=100, threshold2=200)
yellow_lines = cv2.inRange(background, (0, 225, 225), (5, 255, 255))
white_lines = cv2.inRange(background, (250, 250, 250), (255, 255, 255))
cv2.imwrite('./yellow_lines.png', yellow_lines)
cv2.imwrite('./white_lines.png', white_lines)

WIDTH, HEIGHT = background.shape[1::-1]
WIN = pygame.display.set_mode((int(WIDTH/2), int(HEIGHT/2)))
pygame.display.set_caption("Car Test")

# BACKGROUND = pygame.transform.scale(pygame.image.load('./C3_map_2.png'), (int(WIDTH), int(HEIGHT)))


#Debug: print when pass yellow and white lines
yw_messages = True


# Car Initiallization
CAR_WIDTH, CAR_HEIGHT = 30, 30
CAR_IMAGE = pygame.image.load(os.path.join('car.png'))
CAR_DIRECTION = 0
CAR = pygame.transform.rotate(pygame.transform.scale(
    CAR_IMAGE, (CAR_WIDTH//2, CAR_HEIGHT)), CAR_DIRECTION)


# Speed Information
VEL = 5
ANGLE = 4

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
        net_direct += ANGLE
    if keys_pressed[pygame.K_RIGHT]:  # RIGHT
        net_direct += -ANGLE
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
        CAR_DIRECTION)
    car_rect.x += vel * np.sin(CAR_DIRECTION*np.pi/180)
    car_rect.y += vel * np.cos(CAR_DIRECTION*np.pi/180)
    # Check Crashes
    
    return car_rect

def draw_car(bg_copy, x, y, color=(0,0,255)):
    global CAR_DIRECTION
    sin = np.sin(CAR_DIRECTION*np.pi/180)
    cos = np.cos(CAR_DIRECTION*np.pi/180)
    points = np.array([[x+10*(np.sin((CAR_DIRECTION-45)*np.pi/180)), y+10*(np.cos((CAR_DIRECTION-45)*np.pi/180))],
        [x+20*sin, y+20*cos], 
        [x+10*(np.sin((CAR_DIRECTION+45)*np.pi/180)), y+10*(np.cos((CAR_DIRECTION+45)*np.pi/180))],
        [x+10*(np.sin((CAR_DIRECTION+135)*np.pi/180)), y+10*(np.cos((CAR_DIRECTION+135)*np.pi/180))],
        [x+10*(np.sin((CAR_DIRECTION+225)*np.pi/180)), y+10*(np.cos((CAR_DIRECTION+225)*np.pi/180))]])
    cv2.fillPoly(bg_copy, np.int32([points]), color=color)
    return bg_copy

def draw(car_rect):
    bg = draw_car(background.copy(), car_rect.x, car_rect.y)
    pygame.transform.scale(pygame.image.frombuffer(bg.tostring(), bg.shape[1::-1], 'RGB'), (int(WIDTH), int(HEIGHT)))
    WIN.blit(pygame.transform.scale(
        pygame.image.frombuffer(bg.tostring(), bg.shape[1::-1], 'BGR'),
        (int(WIDTH/2), int(HEIGHT/2))), (0,0))

    pygame.display.update()

def check_collisons(x, y, car_rect):
    global WIDTH
    global HEIGHT
    global spawn_x
    global spawn_y
    # Calculate Crop
    half_crop = 50
    min_x = x - half_crop
    min_x = min_x if min_x >= 0 else 0
    min_y = y - half_crop
    min_y = min_y if min_y >= 0 else 0
    max_x = x + half_crop
    max_x = max_x if max_x < WIDTH else WIDTH - 1
    max_y = y + half_crop
    max_y = max_y if max_y < HEIGHT else HEIGHT - 1
    # Use crop to get edges/lines
    grass_crop = grass_edge[min_y:max_y, min_x:max_x]
    yellow_crop = yellow_lines[min_y:max_y, min_x:max_x]
    stop_crop = white_lines[min_y:max_y, min_x:max_x]
    # Create crop of vehicle location
    x_crop = x - min_x
    y_crop = y - min_y
    blank = np.zeros(stop_crop.shape, grass_crop.dtype)
    car_crop = draw_car(blank, x-min_x, y-min_y, color=(255))
    # cv2.imwrite('./car.png', car_crop)
    # cv2.imwrite('./ge.png', grass_crop)
    # cv2.imwrite('./ylw.png', yellow_crop)
    # cv2.imwrite('./stop.png', stop_crop)
    if np.sum(np.bitwise_and(car_crop, grass_crop)):
        car_rect = pygame.Rect(spawn_x, spawn_y, CAR_WIDTH, CAR_HEIGHT)
        print('Car left the road')
    elif np.sum(np.bitwise_and(car_crop, stop_crop)):
        if yw_messages:
            print('Passed a stop line')
    elif np.sum(np.bitwise_and(car_crop, yellow_crop)):
        if yw_messages:
            print('Passed a yellow line')
    return car_rect




def main():
    car_rect = pygame.Rect(spawn_x, spawn_y, CAR_WIDTH, CAR_HEIGHT)

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
        car_rect = check_collisons(car_rect.x, car_rect.y, car_rect)
        draw(car_rect)



if __name__ == '__main__':
    main()