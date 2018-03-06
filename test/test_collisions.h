#ifndef TEST_COLLISIONS_H
#define TEST_COLLISIONS_H

#include "maths.h"
#include "gphyx.h"

class TestCollisions : public SDLTest {
public:

    struct BodyCtx {
        Body body;
        Collider collider;
        Transform transform;
        Speed speed;
    };

    struct BodiesPairHandler : public BodiesPairHandlerBase {
        void init(TightArray<BodyCtx>& bs) {
            bodies = &bs;
        }

        TightArray<BodyCtx>* bodies;
        BodyCtx* bctx1;
        BodyCtx* bctx2;

        void setBodyA(Index ent_id) { BodiesPairHandlerBase::setBodyA(ent_id); bctx1 = &bodies->accItem(ent_id); }
        void setBodyB(Index ent_id) { BodiesPairHandlerBase::setBodyB(ent_id); bctx2 = &bodies->accItem(ent_id); }

        Body& accBodyA() { return bctx1->body; }
        Speed& accSpeedA() { return bctx1->speed; }
        Transform& accTransformA() { return bctx1->transform; }
        Collider& accColliderA() { return bctx1->collider; }

        Body& accBodyB() { return bctx2->body; }
        Speed& accSpeedB() { return bctx2->speed; }
        Transform& accTransformB() { return bctx2->transform; }
        Collider& accColliderB() { return bctx2->collider; }
    };

    TightArray<BodyCtx> bodies;
    PhysicsManagerSimpleT<BodiesPairHandler> pm;

    f32 simulation_time_acc;
    f32 simulation_speed;
    f32 pause_simulation_speed;
    bool step_simulation;

    u32 contacts_count;

    Index attached_body_id;
    Index selected_body_id;
    Vec2 shape_attach_pt;

    F8x8::SDL2Text info_text;

    enum TestType {
        ttTowerSlope,
        ttPyramidSlope,
        ttRevoluteTest,
        ttMegaPyramidTest,
        ttStackingTest,

        ttCount
    };

    enum ShowMode {
        smShapes = BIT_MASK(0),
        smBounds = BIT_MASK(1),
        smSegments = BIT_MASK(2),
        smBoundsWithSegments = smBounds | smSegments,
        smShapesBounds = smShapes | smBounds
    };

    TestType curr_test;
    u32 stack_height;
    f32 stacking_time;
    f32 stacking_y_limit;

    ShowMode show_mode;

    void init(Config::ConfigSectionMap& cfg) {
        simulation_time_acc = 0.0f;
        simulation_speed = 1.0f;
        pause_simulation_speed = 0.0f;
        show_mode = smShapes;
        step_simulation = false;
        contacts_count = 0;
        pm.setUPS(60);
        pm.accBPH().init(bodies);
        stack_height = 10;

        info_text.setColor(0, 255, 255, 255);

        startTest(ttPyramidSlope);

        std::cout << "Controls: o: show/hide profiling info" << std::endl
                  << "          +, -: controll simulation speed" << std::endl
                  << "          p: pause simulation" << std::endl
                  << "          space: step simulation (when paused)" << std::endl
                  << "          up, down: change test" << std::endl
                  << "          s: toggle show mode (shapes, bounds, bounds-with-segments, both)" << std::endl;
    }

    void close() {
        bodies.clear();
        pm.clear();
    }

    void handleEvent(SDL_Event& evt) {
        switch (evt.type) {
            case (SDL_MOUSEBUTTONDOWN): {
                if (evt.button.button == SDL_BUTTON_RIGHT) {
                    attached_body_id = pickBody(Vec2(evt.button.x, evt.button.y));
                }
                else if (evt.button.button == SDL_BUTTON_LEFT) {
                    selected_body_id = pickBody(Vec2(evt.button.x, evt.button.y));
                }
            }break;
            case (SDL_MOUSEBUTTONUP): {
                if (evt.button.button == SDL_BUTTON_RIGHT) {
                    if (attached_body_id.isValid()) {
                        BodyCtx& bctx = bodies.accItem(attached_body_id);
                        // transform to global space
                        Vec2 fvec_start(bctx.transform.calcMatrix()*shape_attach_pt);
                        Vec2 fvec_end(evt.button.x, evt.button.y);
                        Vec2 force_vec = (fvec_end - fvec_start)*pm.getUpdatesPerSec();
                        Vec2 arm = fvec_end - bctx.transform.getPosition();
                        bctx.body.applyForce(arm, force_vec*bctx.body.getMassData().mass);
                        attached_body_id.makeInvalid();
                    }
                }
            }break;
            case (SDL_MOUSEMOTION): {
            }break;
            case (SDL_KEYDOWN): {
                switch (evt.key.keysym.sym) {
                    case SDLK_p: {
                        std::swap(pause_simulation_speed, simulation_speed);
                    }break;
                    case SDLK_SPACE:
                    case 1073741824: {        // from some strange reason SPACE gives this value
                        if (simulation_speed == 0.0f) {
                            step_simulation = true;
                        }
                    }break;
                    case SDLK_PLUS:
                    case SDLK_KP_PLUS: {
                        simulation_speed += 0.1f;
                    }break;
                    case SDLK_MINUS:
                    case SDLK_KP_MINUS: {
                        simulation_speed -= 0.1f;
                        simulation_speed = std::max(0.0f, simulation_speed);
                    }break;
                    case SDLK_UP: {
                        startTest(TestType(wrap((i32)curr_test + 1, ttCount)));
                    }break;
                    case SDLK_DOWN: {
                        startTest(TestType(wrap((i32)curr_test-1, ttCount)));
                    }break;
                    case SDLK_s: {
                        switch (show_mode) {
                            case smShapes: show_mode = smBounds; break;
                            case smBounds: show_mode = smBoundsWithSegments; break;
                            case smBoundsWithSegments: show_mode = smShapesBounds; break;
                            case smShapesBounds: show_mode = smShapes; break;
                            default: break;
                        }
                    }break;
                }
            }break;
            case (SDL_KEYUP): {
            }break;
        }
    }

    void update(SDL_Renderer* r, f32 dt) {
        PROFILE_BLOCK("update");

        if (step_simulation) {
            simulationStep();
            step_simulation = false;
        }
        else {
            simulation_time_acc += simulation_speed*dt;
            while (simulation_time_acc > pm.getUpdateDt()) {
                simulationStep();
                simulation_time_acc -= pm.getUpdateDt();
            }
        }

        {
            PROFILE_BLOCK("draw");
            SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
            SDL_RenderClear(r);

            if (show_mode & smShapes) {
                // draw shapes
                u32 flags = DebugDraw::fShowPosition | DebugDraw::fShowCenter | DebugDraw::fShowRotation;
                SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
                for (u32 i=0; i<bodies.size(); ++i) {
                    const BodyCtx* bctx = bodies.getItemAtPos(i);
                    const Transform& t = bctx->transform;
                    DebugDraw::drawShape(r, bctx->collider.getShape(), t, flags);
                }
                if (selected_body_id .isValid()) {
                    const BodyCtx& bctx = bodies.getItem(selected_body_id);
                    SDL_SetRenderDrawColor(r, 0, 255, 255, 255);
                    const Transform& t = bctx.transform;
                    DebugDraw::drawShape(r, bctx.collider.getShape(), t, flags);
                }
            }

            if (attached_body_id .isValid()) {
                SDL_SetRenderDrawColor(r, 0, 255, 255, 255);
                i32 mouse_x, mouse_y;
                SDL_GetMouseState(&mouse_x, &mouse_y);
                const BodyCtx& bctx = bodies.getItem(attached_body_id);
                Vec2 start(bctx.transform.calcMatrix()*shape_attach_pt);
                Vec2 end(mouse_x, mouse_y);
                DebugDraw::drawArrow(r, start, end);
            }


            info_text.setRenderer(r);
            std::string show_mode_str;
            switch (show_mode) {
                case smShapes: show_mode_str = "Shapes"; break;
                case smBounds: show_mode_str = "Bounds"; break;
                case smBoundsWithSegments: show_mode_str = "Bounds & SAP segments"; break;
                case smShapesBounds: show_mode_str = "Shapes & Bounds"; break;
                default: break;
            }
            std::string it(ssu::formatA("Show Mode: %s\nBodies: %u\nContacts: %u\nSimulation speed: %.3f%s\n",
                                        show_mode_str.c_str(),
                                        u32(bodies.size()),
                                        contacts_count,
                                        simulation_speed,
                                        (pause_simulation_speed > simulation_speed)?" (PAUSED)":""));
            if (selected_body_id .isValid()) {
                const BodyCtx& bctx = bodies.getItem(selected_body_id);
                it += ssu::formatA("\nmass: %.3f\nfriction: %.3f\nrestitution: %.3f\nlin_speed:%.3f\nang_speed:%.3f",
                                   bctx.body.getMassData().mass,
                                   bctx.body.getMaterial().friction,
                                   bctx.body.getMaterial().restitution,
                                   bctx.speed.getLinearSpeed().getLen(),
                                   bctx.speed.getAngularSpeed().getRads()
                                   );
            }
            info_text.setText(it);
            int ww, wh;
            SDL_GetWindowSize(accTestBench().getWindow(), &ww, &wh);
            info_text.draw(ww-info_text.getWidth()-15, 15);

            if (show_mode & smBounds) {
                ARect bounds({0, 0}, {(f32)ww, (f32)wh});
                Color segm_color(Color::Yellow());
                if (!(show_mode & smSegments)) {
                    segm_color.a = 0;
                }
                pm.accSAP().debugRender(accTestBench().getWindow(), r, bounds.getDataPtr(), Color::Green(), segm_color);
            }
        }
    }

    void addSceneBoundary(const Vec2& lefttop, const Vec2& size, f32 thickness) {
        Shape hor_box;
        hor_box.create<Rect>(Vec2(0,0), Vec2(size.getX(), thickness), Vec2(0,0), 0);
        Shape vert_box;
        vert_box.create<Rect>(Vec2(0,0), Vec2(thickness, size.getY()-2*thickness), Vec2(0,0), 0);

        Index body_id;
        // floor
        BodyCtx* bctx = &bodies.add2(body_id);
        bctx->collider.setShape(hor_box);
        bctx->collider.calcTightBound();
        bctx->transform.setPosition({lefttop.getX(), lefttop.getY() + size.getY() - thickness});
        bctx->body.initStatic(Material::Wood());
        pm.addCollider(bctx->collider, bctx->transform, body_id);

        // left wall
        bctx = &bodies.add2(body_id);
        bctx->collider.setShape(vert_box);
        bctx->collider.calcTightBound();
        bctx->transform.setPosition({lefttop.getX(), lefttop.getY()+thickness});
        bctx->body.initStatic(Material::Wood());
        pm.addCollider(bctx->collider, bctx->transform, body_id);

        // right wall
        bctx = &bodies.add2(body_id);
        bctx->collider.setShape(vert_box);
        bctx->collider.calcTightBound();
        bctx->transform.setPosition({lefttop.getX()+size.getX()-thickness, lefttop.getY()+thickness});
        bctx->body.initStatic(Material::Wood());
        pm.addCollider(bctx->collider, bctx->transform, body_id);

        // ceiling
        bctx = &bodies.add2(body_id);
        bctx->collider.setShape(hor_box);
        bctx->collider.calcTightBound();
        bctx->transform.setPosition(lefttop);
        bctx->body.initStatic(Material::Wood());
        pm.addCollider(bctx->collider, bctx->transform, body_id);
    }

    void addSlopeAndBall(const Vec2& p) {
        Vec2 pos = p;
        Shape slope_box;
        slope_box.create<Rect>(Vec2(0,0), Vec2(300, 10), Vec2(0,0), Angle::degrees(40));

        Index body_id;
        BodyCtx* bctx = &bodies.add2(body_id);
        bctx->collider.setShape(slope_box);
        bctx->collider.calcTightBound();
        bctx->transform.setPosition(pos);
        bctx->body.initStatic(Material::Wood());
        pm.addCollider(bctx->collider, bctx->transform, body_id);
        pos += Vec2(20, -50);

        Shape ball;
        ball.create<Circle>(Vec2(0, 0), 20);

        bctx = &bodies.add2(body_id);
        bctx->collider.setShape(ball);
        bctx->collider.calcTightBound();        // because it is circle
        bctx->transform.setPosition(pos);
        bctx->body.initWithShape(Material::Metal(), bctx->collider.getShape());
        pm.addCollider(bctx->collider, bctx->transform, body_id);
    }

    Shape& createRectPgon(Shape& sh, Vec2 rsize) {
        Vec2 pts[] = { -rsize/2, Vec2(rsize.getX()/2, -rsize.getY()/2),
                       rsize/2, Vec2(-rsize.getX()/2, rsize.getY()/2) };
        sh.create<Pgon>(pts, ARRAY_SIZE(pts));
        sh.acc<Pgon>().calculateNormalsIfNeeded();
        return sh;
    }

    void addStackOfBoxes(const Vec2& p, u32 height) {
        Vec2 rsize(30, 15);
        Index rect_sid;
        Shape rect;
        rect.create<Rect>(Vec2(0, 0), rsize, -rsize/2, 0.0f);
        //createRectPgon(rect, rsize);


        Material mat = Material::Wood();
        Index body_id;
        Vec2 pos = p;
        for (u32 i=0; i<height; ++i) {
            BodyCtx* bctx = &bodies.add2(body_id);
            bctx->collider.setShape(rect);
            bctx->collider.calcRotationInvariantBound();        // because it is rectangle
            bctx->transform.setPosition(pos);
            bctx->body.initWithShape(mat, bctx->collider.getShape());
            pm.addCollider(bctx->collider, bctx->transform, body_id);
            pos.accY() -= rsize.accY()*1.1f;
        }
    }

    void addPyramidOfBoxes(const Vec2& p,  u32 height, const Vec2& rsize) {
        Index rect_sid;
        Shape rect;
        rect.create<Rect>(Vec2(0, 0), rsize, -rsize/2, 0.0f);
        //createRectPgon(rect, rsize);

        Material mat = Material::Wood();
        Index body_id;
        Vec2 pos = p;
        for (u32 i=0; i<height; ++i) {
            for (u32 j=(height-i); j>0; --j) {
                BodyCtx* bctx = &bodies.add2(body_id);
                bctx->collider.setShape(rect);
                bctx->collider.calcRotationInvariantBound();        // because it is rectangle
                bctx->transform.setPosition(pos);
                bctx->body.initWithShape(mat, bctx->collider.getShape());
                pm.addCollider(bctx->collider, bctx->transform, body_id);
                pos.accX() += rsize.getX();
            }
            pos.accY() -= rsize.getY()*1.1f;
            pos.accX() = p.getX() + (i+1)*rsize.getX()*0.5f;
        }
    }

    void stackingTest(u32 height) {
        // stacking test
        stack_height = height;
        simulation_speed = 500.0f;
        addStackOfBoxes(Vec2(400, 720), stack_height);
        stacking_time = 0.0f;
        stacking_y_limit = bodies.getItemAtPos(bodies.size()-1)->transform.getPosition().getY() + 100;
    }

    void startTest(TestType tt) {
        curr_test = tt;
        bodies.clear();
        pm.clear();
        simulation_speed = 1.0f;
        simulation_time_acc = 0.0f;
        selected_body_id = attached_body_id = Index::Invalid();

        int ww, wh;
        SDL_GetWindowSize(accTestBench().getWindow(), &ww, &wh);
        addSceneBoundary(Vec2(10, 10), Vec2((f32)ww-20, 750), 10);

        switch (tt) {
            case ttTowerSlope:
                addStackOfBoxes(Vec2(400, 720), 15);
                addSlopeAndBall(Vec2(50, 200));
                break;
            case ttPyramidSlope:
                addPyramidOfBoxes(Vec2(250, 720), 20, Vec2(25, 25));
                addSlopeAndBall(Vec2(50, 200));
                break;
            case ttMegaPyramidTest:
                addPyramidOfBoxes(Vec2(150, 720), 60, Vec2(13, 9));
                break;
            case ttRevoluteTest: {
//                Index b1_id, b2_id;
//                BodyCtx* bctx = &bodies.add2(b1_id);
//                Vec2 size(100, 20);
//                Vec2 pos_off(400, 400);
//                Vec2 pos(0.0f, -0.5f*size.getY());
//                Shape& rect = shapes.add2<Rect>(bctx->shape_id, Vec2(0, 0), size, -size/2, 0);
//                bctx->transform.setPosition(pos_off+pos);
//                pm.initBody(bctx->body, b1_id, Material::Wood(), bctx->transform.getPosition(), rect, 0);
//
//                bctx = &bodies.add2(b2_id);
//                size.set(12.0f, 0.25f);
//                pos.set(0.0f, 1.0f);
//                Shape& plank = shapes.add2<Rect>(bctx->shape_id, Vec2(0, 0), size, -size/2, 0);
//                bctx->transform.setPosition(pos_off+pos);
//                pm.initBody(bctx->body, b2_id, Material::Wood(), bctx->transform.getPosition(), rect, 100);
//                pm.addRevoluteJoint(b1_id, b2_id, pos_off+pos, bph);
                addSlopeAndBall(Vec2(50, 200));

                // add revolute
                Vec2 size(400, 10);
                Vec2 pos(500, 650);
                Shape rect;
                rect.create<Rect>(Vec2(0, 0), size, -size/2, 0);
                Index body_id;
                BodyCtx* bctx = &bodies.add2(body_id);
                bctx->collider.setShape(rect);
                bctx->collider.calcRotationInvariantBound();        // because it is rectangle
                bctx->transform.setPosition(pos);
                bctx->body.initWithShape(Material::Wood(), bctx->collider.getShape());
                pm.addCollider(bctx->collider, bctx->transform, body_id);
                pm.addJoint<RevoluteJoint>(body_id, Index(0, 0), pos);
            }break;
            case ttStackingTest:
                // stacking test
                stackingTest(stack_height);
                break;
            default: break;
        }
    }

    void simulationStep() {
        PROFILE_BLOCK("simulation");

        // update velocities
        for (u32 i=0; i<bodies.size(); ++i) {
            BodyCtx& bctx = *bodies.accItemAtPos(i);
            // add gravity
            bctx.body.applyForceAtCenter(bctx.body.getMassData().mass*Vec2{0.0f, 10});
            pm.updateVelocity(bctx.body, bctx.speed);
        }

        pm.updateContacts();
        contacts_count = pm.getContactsCount();

        // correct velocities with velocity constraints
        pm.solve();

        // update positions
        for (u32 i=0; i<bodies.size(); ++i) {
            BodyCtx& bctx = *bodies.accItemAtPos(i);
            Vec2 move_vec = pm.updateTransform2(bctx.speed, bctx.transform);
            // dynamic bodies do not change scale,
            // their colliders are rotation invariant, we only move boxes in SAP
            pm.moveCollider(bctx.collider, move_vec);
        }


        if (curr_test == ttStackingTest && stacking_time >= 0.0f) {
            stacking_time += pm.getUpdateDt();
            if (stacking_time >= 1000.0f) {
                stack_height +=1;
                startTest(ttStackingTest);
            }
            else if (bodies.getItemAtPos(bodies.size()-1)->transform.getPosition().getY() > stacking_y_limit) {
                std::cout << "stack of " << stack_height << " boxes fell after " << stacking_time << " secs." << std::endl;
                stacking_time = -1.0f;
                simulation_speed = 1.0f;
                stack_height = 10;
            }
        }
    }

    Index pickBody(const Vec2& pos) {
        Index body_id;
        for (u32 i = 0; i < bodies.size(); ++i) {
            const BodyCtx* bctx = bodies.getItemAtPos(i);
            shape_attach_pt = (-bctx->transform).calcMatrix() * pos;
            if (bctx->collider.getShape().isPointInside(shape_attach_pt)) {
                body_id = bodies.getIndexForPos(i);
                break;
            }
        }
        return body_id;
    }
};

#endif //TEST_COLLISIONS_H
