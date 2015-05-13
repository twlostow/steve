#include <cstdio>
#include <cstdlib>

#include <geometry/shape.h>
#include <geometry/shape_rect.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_segment.h>
//#include <geometry/shape_convex.h>

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>

#include <boost/foreach.hpp>

#include <stdio.h>
#include <stdlib.h>

#include <string>

using namespace std;

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

struct MyShape {
	MyShape() :
		hover(false),
		dragging(false),
        shape(NULL)
        {
			color.r= 255; color.g=0; color.b= 0;
			alpha = 255;
		}

        MyShape (const MyShape &b)
        {
            shape = b.shape->Clone();
            hover = false;
            dragging =false;
            color = b.color;
        }

        ~MyShape()
        {
            if(shape)
                delete shape;
        }

        bool hitTest (int x, int y)
        {
           int thr = shape->IsSolid() ? 0 : 5;
            return shape->Collide( VECTOR2I(x,y), thr );
	    }

	   SHAPE *shape;
	   bool hover;
	   bool dragging;
       bool colliding;
	   VECTOR2I dragOrigin;
	   Color color;
	   int alpha;
       VECTOR2I collPoint;
};

SDL_Surface *screen;
vector<MyShape *> shapes;

bool inSet(char c, const char *str)
{
	while(*str) { if (*str == c) return true; str++; }
	return false;
}

string getToken ( string& s, int t )
{
	int n = 0, i = 0;
	string rv;

	for(;;) {
		while(i < s.length() && inSet(s[i]," \t\n\r")) i++;
		if( i== s.length() )
			break;

		while(i < s.length() && !inSet(s[i]," \t\n\r")) 
			{
				if(n==t)
					rv += s[i];
				i++;
			}
		
		if(i==s.length()) 
			break;
		n++;
	}
	return rv;
}

int getIntToken (string& s, int t)
{
	return atoi ( getToken(s, t). c_str() );
}


void readShapes( const string& fname)
{
	FILE *f=fopen(fname.c_str(), "rb");
    int n = 0;
	while(!feof(f))
	{
		char tmp[16384];
		fgets(tmp, 16484, f);
		string line(tmp);

		string type = getToken(line, 0);

        MyShape *s = new MyShape;
		
		if(type == "circle")
			s->shape = new SHAPE_CIRCLE ( VECTOR2I ( getIntToken(line, 1), getIntToken(line, 2) ), getIntToken(line, 3) );
		if(type == "rect")
			s->shape = new SHAPE_RECT ( VECTOR2I ( getIntToken(line, 1), getIntToken(line, 2) ), getIntToken(line, 3), getIntToken(line, 4) );
		if(type == "line_chain")
        {
            SHAPE_LINE_CHAIN *lc = new SHAPE_LINE_CHAIN();
            int n = getIntToken(line ,1);
            bool closed = getIntToken(line, 2);

            for(int i = 0; i<n;i++)
                lc->Append (getIntToken(line, 3+i*2), getIntToken(line, 3+i*2+1));
            s->shape = lc;
        }
        if(type == "segment")
        {
            s->shape = new SHAPE_SEGMENT ( VECTOR2I ( getIntToken(line, 2), getIntToken(line, 3) ), VECTOR2I(getIntToken(line, 4), getIntToken(line, 5)), getIntToken(line, 1) );
        }
#if 0
        if(type == "convex")
        {
            SHAPE_CONVEX *cx = new SHAPE_CONVEX();
            int n = getIntToken(line ,1);
            
            for(int i = 0; i<n;i++)
                cx->Append (getIntToken(line, 2+i*2), getIntToken(line, 2+i*2+1));
            s->shape = cx;
        }
#endif
            
            
        
        if(s->shape)
        {
            s->color = Color(n++);
            shapes.push_back(s);
        }
        else delete s;
	}

}


void drawShape (const MyShape *sh)
{
    const SHAPE *s = sh->shape;
    Color col = sh->color;

    if(sh->hover)
	   col = col.Lighter(0.7);

    if(sh->colliding)
	   col.r = 255.0;

    switch(s->Type())
    {
		case SH_CIRCLE:
		{
			const SHAPE_CIRCLE *c = static_cast<const SHAPE_CIRCLE *> (s);
			filledEllipseRGBA(screen,
				c->GetCenter().x, c->GetCenter().y,
				c->GetRadius(), c->GetRadius(),
				col.r, col.g, col.b, col.a );

			ellipseRGBA(screen,
				c->GetCenter().x, c->GetCenter().y,
				c->GetRadius(), c->GetRadius(),
				192,192,192, sh->alpha );

			
			break;
		}

		case SH_RECT:
		{
			const SHAPE_RECT *c = static_cast<const SHAPE_RECT *> (s);
			boxRGBA(screen,
				c->GetPosition().x, c->GetPosition().y,
				c->GetPosition().x + c->GetSize().x, c->GetPosition().y + c->GetSize().y,
				col.r, col.g, col.b, col.a);
			rectangleRGBA(screen,
				c->GetPosition().x, c->GetPosition().y,
				c->GetPosition().x + c->GetSize().x, c->GetPosition().y + c->GetSize().y,
				192,192,192, col.a );
			
			break;
		}

        case SH_LINE_CHAIN:
        {
            const SHAPE_LINE_CHAIN *lc = static_cast<const SHAPE_LINE_CHAIN *> (s);
            int n = lc->SegmentCount();


            for(int i = 0; i < n; i++)
            {
                const SEG s = lc->CSegment(i);
                lineRGBA( screen, s.A.x, s.A.y, s.B.x, s.B.y, col.r, col.g, col.b, 220 );
            }
        }

        case SH_SEGMENT:
        {
            const SHAPE_SEGMENT *seg = static_cast<const SHAPE_SEGMENT *> (s);
            const SEG ss = seg->GetSeg();
            int w = seg->GetWidth();
            thickLineRGBA( screen, ss.A.x, ss.A.y, ss.B.x, ss.B.y, w, col.r, col.g, col.b, col.a );
            filledEllipseRGBA(screen, ss.A.x, ss.A.y, w/2, w/2, col.r, col.g, col.b, col.a );
            filledEllipseRGBA(screen, ss.B.x, ss.B.y, w/2, w/2, col.r, col.g, col.b, col.a );
            break;
        }

#if 0
        case SH_CONVEX:
        {
            const SHAPE_CONVEX *lc = static_cast<const SHAPE_CONVEX *> (s);
            int n = lc->SegmentCount();
            Sint16 xt[128],yt[128];


            for(int i =0 ; i < lc->PointCount(); i++ )
            {
                xt[i] = lc->CPoint(i).x;
                yt[i] = lc->CPoint(i).y;
            }

            filledPolygonRGBA(screen, xt, yt, lc->PointCount(), col.r, col.g, col.b, col.a);

            for(int i = 0; i < n; i++)
            {
                const SEG s = lc->CSegment(i);
                lineRGBA( screen, s.A.x, s.A.y, s.B.x, s.B.y, 192, 192, 192, col.a );
            }   

        }
#endif
	    
    }
}

void testCollisions()
{
    int clearance = 10;
    for(int i = 0; i < shapes.size(); i++)
        shapes[i]->colliding = false;

    for(int i = 0; i < shapes.size(); i++)
        for(int j = i + 1; j < shapes.size(); j++)
        {
            MyShape *s1 = shapes[i];
            MyShape *s2 = shapes[j];
            VECTOR2I mtv;
            if (CollideShapes(s2->shape, s1->shape, clearance, true, mtv))
            {  
                s1->colliding = true;
                s2->colliding = true;
                s1->collPoint = s2->shape->Centre();

                VECTOR2I a (s1->shape->Centre());
                VECTOR2I b (s2->shape->Centre());
                lineRGBA( screen, a.x, a.y, b.x, b.y, 255, 128,128, 255 );
            
                if(mtv.x || mtv.y)
                {
                    MyShape tmp ( *s1 );
                    
                    printf("mtv %d, %d\n", mtv.x, mtv.y);

                    tmp.shape->Move ( mtv );
                    tmp.color = Color(128,128,128,255);
                    
                    drawShape(&tmp);
                }

            }
        }
}


SHAPE_LINE_CHAIN makeCircleQuad ( const VECTOR2I start, const VECTOR2I dir, int sign, int segCount = 10 )
{

    SHAPE_LINE_CHAIN lc;

    for (int i = 0; i <= segCount; i ++)
    {
        double alpha = (double) (i) / (double) segCount * M_PI / 2.0;
    
    }

}

/*
enum MeanderStyle {
    MEANDER_S = 0, MEANDER_Z, MEANDER_J 
}

void makeMeander ( const VECTOR2I &start, const VECTOR2I &dir, int minAmplitude, int maxAmplitude, int spacing )
{   

}
*/

static void quit_tutorial( int code )
{
    /*
     * Quit SDL so we can release the fullscreen
     * mode and restore the previous video settings,
     * etc.
     */
    SDL_Quit( );

    /* Exit program. */
    exit( code );
}

static void handle_key_down( SDL_keysym* keysym )
{

    /* 
     * We're only interested if 'Esc' has
     * been presssed.
     *
     * EXERCISE: 
     * Handle the arrow keys and have that change the
     * viewing position/angle.
     */
    switch( keysym->sym ) {
    case SDLK_ESCAPE:
        quit_tutorial( 0 );
        break;
    case SDLK_SPACE:
//        should_rotate = !should_rotate;
        break;
    default:
        break;
    }

}


void handle_motion( int x, int y)
{
    BOOST_FOREACH(MyShape *s, shapes)
    {
	   s->hover = s->hitTest(x, y);
       VECTOR2I p (x, y);
       
       
       if(s->dragging)
            s->shape->Move ( p - s->dragOrigin );
       s->dragOrigin = p;
    }
}

void handle_press()
{
    BOOST_FOREACH(MyShape *s, shapes)
    {
       s->dragging = s->hover;
    }
}

void handle_release()
{
    BOOST_FOREACH(MyShape *s, shapes)
    {
       s->dragging = false;
    }
}



static void process_events( void )
{
    /* Our SDL event placeholder. */
    SDL_Event event;

    /* Grab all the events off the queue. */
    while( SDL_PollEvent( &event ) ) {

        switch( event.type ) {
        case SDL_KEYDOWN:
            /* Handle key presses. */
            handle_key_down( &event.key.keysym );
            break;

        case SDL_MOUSEBUTTONUP:
            handle_release();
            //handle_motion( event.motion.x, event.motion.y );
           break;

        case SDL_MOUSEBUTTONDOWN:
            handle_press();
            //handle_motion( event.motion.x, event.motion.y );
           break;


	   case SDL_MOUSEMOTION:
	        handle_motion( event.motion.x, event.motion.y );
            break;

        case SDL_QUIT:
            /* Handle quit requests (like Ctrl-c). */
            quit_tutorial( 0 );
            break;
        }

    }

}

static void draw_screen( void )
{
	for(int i=0;i<shapes.size();i++)
		drawShape(shapes[i]);
    
}


int main( int argc, char* argv[] )
{
    /* Dimensions of our window. */
    int width = 0;
    int height = 0;
    /* Color depth in bits of our window. */
    int bpp = 0;
    /* Flags we will pass into SDL_SetVideoMode. */
    int flags = 0;

    readShapes("test_shapes.txt");

    /* First, initialize SDL's video subsystem. */
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
        /* Failed, exit. */
        fprintf( stderr, "Video initialization failed: %s\n",
             SDL_GetError( ) );
        quit_tutorial( 1 );
    }

    width = 1024;
    height = 768;
    bpp = 32;


    screen = SDL_SetVideoMode( width, height, bpp, flags );
    if(screen == NULL) {
        /* 
         * This could happen for a variety of reasons,
         * including DISPLAY not being set, the specified
         * resolution not being available, etc.
         */
        fprintf( stderr, "Video mode set failed: %s\n",
             SDL_GetError( ) );
        quit_tutorial( 1 );
    }

  
    while( 1 ) {
        boxRGBA(screen,0,0, screen->w-1, screen->h-1, 0, 0, 0, 255);
           /* Process incoming events. */
        /* Draw the screen. */
        draw_screen( );
        process_events( );
        testCollisions();
           
        SDL_Flip(screen);
    }

    /*
     * EXERCISE:
     * Record timings using SDL_GetTicks() and
     * and print out frames per second at program
     * end.
     */

    /* Never reached. */
    return 0;
}