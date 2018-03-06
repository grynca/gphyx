#ifndef TEST_DYNAMICS_H
#define TEST_DYNAMICS_H

#include "maths.h"
#include "gphyx.h"

Vec2 points1[] = {
        {-33, -42}, {38, -63}, {62, -12}, {44, 32}, {-44, 54}, {-57, 28}
};

Vec2 points2[] = {
        {-43, 11}, {-26, -20}, {2, -31}, {69, -13}, {-19, 37}
};

class TestDynamics : public SDLTest {
public:
    TightArray<Shape> shapes;
    fast_vector<Index> shape_indices;

    struct BodyCtx {
        u32 shape_id;
        Body body;
        Transform transform;
        Speed speed;
    } bctx;
    F8x8::SDL2Text info_text;
    f32 impulse_push_time;
    Vec2 force_vec;

    bool shape_attached;
    Vec2 shape_attach_pt;

    const Shape& getCurrShape()const {
        return shapes.getItem(shape_indices[bctx.shape_id]);
    }
    Shape& accCurrShape() {
        return shapes.accItem(shape_indices[bctx.shape_id]);
    }


    void init(Config::ConfigSectionMap& cfg) {
        f32 radius = 100;
        Index id;
        shapes.add2(id).create<Circle>(Vec2(0, 0), radius);
        shape_indices.push_back(id);

        Vec2 rsize(300, 150);
        shapes.add2(id).create<Rect>(Vec2(0, 0), rsize, -rsize/2, 0);
        shape_indices.push_back(id);

        shapes.add2(id).create<Rect>(Vec2(0, 0), rsize, Vec2(0,0), 0);
        shape_indices.push_back(id);

        shapes.add2(id).create<Pgon>(points1, ARRAY_SIZE(points1));
        shape_indices.push_back(id);
        Pgon& pgon1 = shapes.accItem(shape_indices.back()).acc<Pgon>();
        Vec2 center = pgon1.calcCenter();
        pgon1.loopPoints([&center](Vec2& pt) {
            // center on origin & scale
            pt -= center;
            pt *= 2.0f;
        });
        pgon1.calculateNormalsIfNeeded();

        shapes.add2(id).create<Pgon>(points2, ARRAY_SIZE(points2));
        shape_indices.push_back(id);
        Pgon& pgon2 = shapes.accItem(shape_indices.back()).acc<Pgon>();
        center = pgon2.calcCenter();
        pgon2.loopPoints([&center](Vec2& pt) {
            // center on origin & scale
            pt -= center;
            pt *= 2.5f;
        });
        pgon2.calculateNormalsIfNeeded();

        int ww, wh;
        SDL_GetWindowSize(accTestBench().getWindow(), &ww, &wh);
        bctx.shape_id = 0;
        bctx.body.setMassData(getCurrShape().calcInertia(), 1);
        bctx.transform.setPosition(Vec2(ww, wh)*0.5f);
        info_text.setColor(0, 255, 255, 255);
        bctx.speed.setLinearSpeed({0, 0});
        bctx.speed.setAngularSpeed(0.0f);

        shape_attached = false;

        std::cout << "Controls: o: show/hide profiling info" << std::endl
                  << "          up,down: change shapes" << std::endl
                  << "          r: reset shape state" << std::endl
                  << "          +, -: change mass" << std::endl;
    }

    void close() {

    }

    void handleEvent(SDL_Event& evt) {
        switch (evt.type) {
            case (SDL_MOUSEBUTTONDOWN): {
                if (evt.button.button == SDL_BUTTON_RIGHT) {
                    shape_attached = false;
                    // transform to shape space
                    shape_attach_pt = (-bctx.transform).calcMatrix() * Vec2(evt.button.x, evt.button.y);
                    if (getCurrShape().isPointInside(shape_attach_pt)) {
                        shape_attached = true;
                    }
                }
            }break;
            case (SDL_MOUSEBUTTONUP): {
                if (evt.button.button == SDL_BUTTON_RIGHT) {
                    if (shape_attached) {
                        shape_attached = false;
                        // transform to global space
                        Vec2 fvec_end(bctx.transform.calcMatrix() * shape_attach_pt);
                        Vec2 fvec_start(evt.button.x, evt.button.y);
                        force_vec = -(fvec_end - fvec_start) * 10;
                        impulse_push_time = 0.1f;
                        Vec2 arm = fvec_end - bctx.transform.getPosition();
                        bctx.body.applyForce(arm, force_vec);
                    }
                }
            }break;
            case (SDL_MOUSEMOTION): {
            }break;
            case (SDL_KEYDOWN): {
                switch (evt.key.keysym.sym) {
                    case SDLK_UP:
                        bctx.shape_id = (u32)wrap(i32(bctx.shape_id-1), i32(shape_indices.size()));
                        bctx.body.setMassData(getCurrShape().calcInertia(), bctx.body.getMassData().mass);
                        break;
                    case SDLK_DOWN:
                        bctx.shape_id = (u32)wrap(i32(bctx.shape_id+1), i32(shape_indices.size()));
                        bctx.body.setMassData(getCurrShape().calcInertia(), bctx.body.getMassData().mass);
                        break;
                    case SDLK_r: {
                        int ww, wh;
                        SDL_GetWindowSize(accTestBench().getWindow(), &ww, &wh);
                        bctx.transform = Transform();
                        bctx.transform.setPosition(Vec2(ww, wh) * 0.5f);
                        bctx.speed.setLinearSpeed({0, 0});
                        bctx.speed.setAngularSpeed(0.0f);
                    }break;
                    case SDLK_KP_PLUS:
                    case SDLK_PLUS:
                        bctx.body.setMassData(getCurrShape().calcInertia(), bctx.body.getMassData().mass +1);
                        break;
                    case SDLK_KP_MINUS:
                    case SDLK_MINUS:
                        bctx.body.setMassData(getCurrShape().calcInertia(), bctx.body.getMassData().mass -1);
                        break;
                }
            }break;
            case (SDL_KEYUP): {

            }break;
        }
    }

    void update(SDL_Renderer* r, f32 dt) {
        PROFILE_BLOCK("update");

        if (impulse_push_time > 0.0f) {
            impulse_push_time -= dt;
            if (impulse_push_time <= 0.0f) {
                bctx.body.clearForces();
            }
        }


        Vec2 lin_acc;
        f32 ang_acc;
        bctx.body.forceToAcceleration(lin_acc, ang_acc);
        bctx.speed.accLinearSpeed() += lin_acc*dt;
        bctx.speed.accAngularSpeed() += ang_acc*dt;
        bctx.transform.rotate(bctx.speed.getAngularSpeed()*dt);
        bctx.transform.move(bctx.speed.getLinearSpeed()*dt);

        SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
        SDL_RenderClear(r);

        SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
        DebugDraw::drawShape(r, accCurrShape(), bctx.transform, DebugDraw::fAll);

        info_text.setRenderer(r);
        info_text.setText(ssu::formatA("Mass: %.3f\nInertia: %.3f\nLin.vel: %.3f [%.3f, %.3f]\nAng.vel: %.3f",
                                       bctx.body.getMassData().mass, bctx.body.getMassData().inertia,
                                       bctx.speed.getLinearSpeed().getLen(),
                                       bctx.speed.getLinearSpeed().getX(),
                                       bctx.speed.getLinearSpeed().getY(),
                                       bctx.speed.getAngularSpeed()
        ));
        int ww, wh;
        SDL_GetWindowSize(accTestBench().getWindow(), &ww, &wh);
        info_text.draw(ww-info_text.getWidth()-5, 5);

        if (shape_attached) {
            i32 mouse_x, mouse_y;
            SDL_GetMouseState(&mouse_x, &mouse_y);
            Vec2 start(bctx.transform.calcMatrix()*shape_attach_pt);
            Vec2 end(mouse_x, mouse_y);
            DebugDraw::drawArrow(r, start, end);
        }
    }
};

#endif //TEST_DYNAMICS_H
