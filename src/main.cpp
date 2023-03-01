#include <cstdlib>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

struct circle{
    float x_position;
    float y_position;

    circle(float x, float y){
        x_position = x;
        y_position = y;
    }

    circle() : x_position(0), y_position(0) {}

    void update_position(){
         x_position += -0.01;
         y_position += -0.01;
    }

    void vitesse(){
            // a remplir
    }
};

int main(int argc, char* argv[])
{
    { // Run the tests
        if (doctest::Context{}.run() != 0)
            return EXIT_FAILURE;
        // The CI does not have a GPU so it cannot run the rest of the code.
        const bool no_gpu_available = argc >= 2 && strcmp(argv[1], "-nogpu") == 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (no_gpu_available)
            return EXIT_SUCCESS;
    }

    // Actual app
    auto ctx = p6::Context{{.title = "Simple-p6-Setup"}};
    ctx.maximize_window();

    circle circle1(-1, 0.5);

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::AppleGreen);
        ctx.circle({-0.5f, 0.5f},
            //p6::Center{ctx.mouse()},
            p6::Radius{0.2f}
        );


        ctx.circle({circle1.x_position, circle1.y_position},
        p6::Radius{0.2f}
        );
        circle1.update_position();

        // ctx.circle({p6::random::number(-ctx.aspect_ratio(), ctx.aspect_ratio()), p6::random::number(-1.f, 1.f)},
        // p6::Radius{0.2f}
        // );

                // ctx.circle({p6::random::number(-ctx.aspect_ratio(), ctx.aspect_ratio()), p6::random::number(-1.f, 1.f)},
        // p6::Radius{0.2f}
        // );
    };

    // SALUUUUUUUT ANAST
    // SALUUUUUUUT MATHILDE

    // Should be done last. It starts the infinite loop.
    ctx.start();
}