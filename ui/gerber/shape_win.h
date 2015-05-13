#ifndef __SHAPE_WIN_H
#define __SHAPE_WIN_H

#include <cstdio>
#include <cstdlib>

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>

#include <boost/foreach.hpp>

#include <stdio.h>
#include <stdlib.h>

#include <string>

struct Color {
    Color(): r(0.0), g(0.0), b(0.0), a(255.0) {}

    Color (int _r, int _g, int _b, int _a)
{
    r = _r; g = _g; b = _b; a= _a;
}

    Color(int n)
    {
	static struct { int r,g,b; } tab [] = {
	{128, 0, 0},
	{0, 128, 0},
	{0, 128, 128},
	{128, 128, 0},
	{0, 128, 128},
	{128, 128, 0},
	{128, 128, 128}};
	a=128.0;
	if(n<7)
	{
    	    r=tab [n].r;
    	    g=tab [n].g;
    	    b=tab [n].b;
	}
    }

    Color Lighter(double factor)
    {
	Color t;
	t.r = r * (1.0-factor) + 255.0 * factor;
	t.g = g * (1.0-factor) + 255.0 * factor;
	t.b = b * (1.0-factor) + 255.0 * factor;
	t.a = a;
	return t;
    }

    double r, g, b, a;
};

struct Shape {

    Shape() :
		hover(false),
		dragging(false),
        shape(NULL)
        {
			color.r= 255; color.g=0; color.b= 0;
			alpha = 255;
		}

        Shape (const Shape &b)
        {
            hover = false;
            dragging =false;
            color = b.color;
        }

        virtual ~Shape()
        {

        }

        virtual bool HitTest (int x, int y)=0;
        virtual void Draw(SDL_Surface *surf)=0;


	   bool hover;
	   bool dragging;
       bool colliding;
	   Color color;
	   int alpha;
};


class SDLView {
    public:
        SDLView(int w, int h);
        ~SDLView();

        void AddShape ( Shape* s);
        void Update();



    private:
        SDL_Surface *screen;

        double scale;
        double cx, cy;
        std::vector<Shape*> shapes;
};

#endif
