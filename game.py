
import random
import math

import pygame
from pygame import gfxdraw
import serial

# size of square maze
mazeSize = 20
cellSize = 40 # number of pixels per "cell"

startPos = (math.floor(mazeSize/2),0)

#Ball start variables
ballStart = (cellSize *.7, math.floor(mazeSize/2)*cellSize - 3)

endPos = (random.randint(0, mazeSize - 1), (mazeSize - 1))
grid = [[0b1111 for i in range(mazeSize)] for x in range(mazeSize)]

board_address = '/dev/cu.usbmodemSDA6C081E511'

_ser = serial.Serial(board_address, 115200, timeout=0.02) # UPDATE THIS to match usb address on your computer


walls = pygame.sprite.Group() # list of segment object representing the walls, used for collision detection

#Maze Generation code ---------

#randomized depth first search
def RDFS():
    visited = [[False for i in range(mazeSize)] for x in range(mazeSize)]
    y, x = startPos
    recursive_DFS(y, x, visited)

def recursive_DFS(y,x, visited):
    visited[y][x] = True
    Pneighbors = [(y,x+1), (y, x-1), (y+1, x), (y-1, x)]   # potential neigbors
    random.shuffle(Pneighbors)
    neighbors = []

    #adding actual in bound and unvisited neighbors
    for ny, nx in Pneighbors:
        if ((mazeSize - 1) >= nx >= 0) and ((mazeSize - 1) >= ny >= 0) and visited[ny][nx] is False:
            neighbors.append((ny, nx))

    #recursive loop
    while neighbors:
        random.shuffle(neighbors)
        next = random.choice(neighbors)
        ny, nx = next

        neighbors.remove((ny, nx))

        if visited[ny][nx]: # needed because neighbors is static once created to ensure no double visits
            continue

        removeWall(y,x, ny, nx)
        recursive_DFS(ny, nx, visited)


    return

def build_walls():
    # add wall segments
    NORTH, SOUTH, EAST, WEST = 0b0001, 0b0010, 0b0100, 0b1000

    for y in range(mazeSize):
        for x in range(mazeSize):

            tile = grid[y][x]

            if tile & NORTH:
                wall = pygame.sprite.Sprite()
                wall.rect = pygame.Rect(x * cellSize+10, y * cellSize+10, cellSize, 1)
                walls.add(wall)
            if tile & SOUTH:
                wall = pygame.sprite.Sprite()
                wall.rect = pygame.Rect(x*cellSize+10, y * cellSize+10+cellSize, cellSize, 1)
                walls.add(wall)
            if tile & EAST:
                wall = pygame.sprite.Sprite()
                wall.rect = pygame.Rect(x * cellSize + cellSize+10, y * cellSize+10, 1, cellSize)
                walls.add(wall)
            if tile & WEST:
                wall = pygame.sprite.Sprite()
                wall.rect = pygame.Rect(x * cellSize+10, y * cellSize+10, 1, cellSize)
                walls.add(wall)


#helper function to remove wall
def removeWall(y,x, ny, nx):
    NORTH, SOUTH, EAST, WEST = 0b0001, 0b0010, 0b0100, 0b1000

    direction = {
        (1,0) : (SOUTH, NORTH),         # wall to knockdown at old tile, wall to knockdown on new tile
        (0,1): (EAST, WEST),
        (-1,0): (NORTH, SOUTH),
        (0,-1): (WEST, EAST)
    }

    dx = nx - x
    dy = ny - y

    oldWall, newWall = direction[dy, dx]

    grid[y][x] &= ~oldWall
    grid[ny][nx] &= ~newWall

## GUI CODE -----------

def drawMaze(grid, screen):
    screen.fill((0,0,0))

    NORTH, SOUTH, EAST, WEST = 0b0001, 0b0010, 0b0100, 0b1000
    for y in range(mazeSize):
        for x in range(mazeSize):

            px = x * cellSize + 10
            py = y * cellSize + 10
            cell = grid[y][x]

            if (y,x) == endPos:
                ey,_ = endPos
                pygame.draw.rect(screen, (158, 46, 46), pygame.Rect((x*cellSize,y*cellSize,cellSize,cellSize)))
            if cell & NORTH:
                pygame.draw.line(screen, (255,255,255), (px,py),(px+cellSize, py),1)
            if cell & SOUTH:
                pygame.draw.line(screen, (255,255,255), (px, py+cellSize), (px+cellSize, py+cellSize), 1)
            if cell & WEST:
                pygame.draw.line(screen, (255,255,255), (px,py),(px,py+cellSize),1)
            if cell & EAST:
                pygame.draw.line(screen, (255,255,255),(px+cellSize, py), (px+cellSize, py+cellSize),1)


#Marble rendering and physics code---------------

def circle_wall_collide(rect,radius, wall):
    cx, cy = rect.center
    closestx = max(wall.rect.left,min(wall.rect.right,cx))
    closesty = max(wall.rect.top, min(wall.rect.bottom,cy))
    dx, dy = cx - closestx, cy - closesty

    overlap = 0
    if dx*dx +dy*dy <= radius*radius:
        overlap = abs(math.sqrt(dx*dx + dy*dy) - radius)

    return overlap, dx, dy


class Marble(pygame.sprite.Sprite):
    def __init__(self, start_pos, radius):
        super().__init__()
        self.image = pygame.Surface((radius * 2, radius*2), pygame.SRCALPHA)

        #provided by chatGPT
        for r in range(radius, 0,-1):
            t = r / radius

            shade = int(105* t + 65)
            alpha = int(255 * (1- .2*t))
            color = (shade, shade, 255, alpha)

            pygame.gfxdraw.filled_circle(self.image, radius,radius, r, color)
        pygame.gfxdraw.aacircle(self.image, radius, radius, radius, (0, 0, 0))
        pygame.gfxdraw.aacircle(self.image, radius, radius, radius-1, (0, 0, 0))


        self.rect = self.image.get_rect(center=start_pos)
        self.pos = pygame.math.Vector2(start_pos)
        self.vel = pygame.math.Vector2(0,0)
        self.radius = radius

    def update(self, dt, acel):
        self.vel += dt*acel
        self.vel.x = pygame.math.clamp(self.vel.x, -200, 200)
        self.vel.y = pygame.math.clamp(self.vel.y, -200, 200)
        self.pos += self.vel *dt
        self.rect.center = round(self.pos.x), round(self.pos.y)
        for w in walls:
            overlap, dx, dy = circle_wall_collide(self.rect, self.radius, w)
            distance = math.hypot(dx,dy)
            if overlap > 0:
                if w.rect.width == 1:
                    self.vel.x *= -.4
                else:
                    self.vel.y *= -.4
                nx, ny = dx / distance, dy / distance
                self.pos += nx*overlap , ny*overlap
                self.rect.center = round(self.pos.x), round(self.pos.y)



def get_accel():
    line = _ser.readline().decode(errors='ignore').strip()
    if not line:
        return pygame.math.Vector2()
    try:
        ax_raw, ay_raw = map(int, line.split(','))

        if -6 < ax_raw <6 and - 6< ay_raw < 6:
            return pygame.math.Vector2(0,0)
        else:
            return pygame.math.Vector2(ay_raw, ax_raw)
    except ValueError:
        return pygame.math.Vector2()

marble = Marble(ballStart, 13)
allSprite = pygame.sprite.Group(marble)
def main():
    pygame.init()

    screen = pygame.display.set_mode((mazeSize * cellSize + 20, mazeSize * cellSize + 20))

    pygame.display.set_caption("maze generation test")

    clock = pygame.time.Clock()

    RDFS() # generate the maze
    build_walls()

    running = True



    while running:

        dt = clock.tick(60) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        acel = get_accel() * 2.5
        print(acel)

        allSprite.update(dt, acel)

        drawMaze(grid,screen)
        allSprite.draw(screen)
        pygame.display.flip()

        mx, my = marble.pos
        ny, nx = endPos

        if mx >= nx*cellSize and (ny*cellSize <= my <= ny*cellSize+cellSize):
            running = False


    #TODO add endgame menu and fix end tile visuals 
    pygame.quit()

if __name__ == "__main__":
    main()
