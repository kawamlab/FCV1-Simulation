#include <stdio.h>
#include <iostream>
#include <pybind11/pybind11.h>
#include <nlohmann/json.hpp>
#include <pybind11/numpy.h>
#include <box2d/box2d.h>
#include <cmath>
#include <limits>
#include <pybind11/stl.h>
#include "fcv1_simulator.hpp"
#include <set>
#include <string>
#include <fstream>
#include <algorithm>


// constexpr float stone_radius = 0.145f;

json read_configfile(const std::string& filepath)
{
    std::ifstream ifs(filepath);
    json j;
    ifs >> j;
    return j;
}

// 返り値1つめ: 正規化されたベクトル
// 返り値2つめ: もとのベクトルの長さ
/// \brief To normalize the vector
/// \param[in] v The vector to be normalized
/// \returns A pair of the normalized vector and the length of the original vector
inline std::pair<b2Vec2, float> normalize(b2Vec2 const &v)
{
    b2Vec2 normalized = v;
    float length = normalized.Normalize();
    return {normalized, length};
}

/// \brief To calculate the longitudinal acceleration
/// \param[in] speed The speed of the stone
/// \returns The longitudinal acceleration
inline float longitudinal_acceleration(float speed)
{
    constexpr float kGravity = 9.80665f;
    return -(0.00200985f / (speed + 0.06385782f) + 0.00626286f) * kGravity;
}

/// \brief To calculate the yaw rate
/// \param[in] speed The speed of the stone
/// \param[in] angularVelocity The angular velocity of the stone
/// \returns The yaw rate
inline float yaw_rate(float speed, float angularVelocity)
{
    if (std::abs(angularVelocity) <= EPSILON)
    {
        return 0.f;
    }
    return (angularVelocity > 0.f ? 1.0f : -1.0f) * 0.00820f * std::pow(speed, -0.8f);
}

/// \brief To calculate the angular acceleration
/// \param[in] linearSpeed The speed of the stone
/// \returns The angular acceleration
inline float angular_acceleration(float linearSpeed)
{
    float clampedSpeed = std::max(linearSpeed, 0.001f);
    return -0.025f / clampedSpeed;
}

py::array_t<double> convert_stonedata(const std::vector<digitalcurling3::StoneData>& simulated_stones, int simulations) {

    // 1次元配列を作成（各ストーンのx, y座標を順に格納）
    size_t simulation_count = static_cast<size_t>(simulations);
    size_t total_size = simulation_count * stones_per_simulation * num_coordinates;
    std::vector<double> temp_result (total_size, 0.0);

    // 1次元配列にデータをコピー
    for (size_t sim = 0; sim < simulation_count; ++sim) {
        size_t base_index = static_cast<size_t>(sim) * stones_per_simulation; // sim * 16

        // team0
        for (int i = 0; i < 8; ++i) {
            temp_result[((sim * 2 + 0) * 8 + i) * 2 + 0] = simulated_stones[base_index + i].position.x;
            temp_result[((sim * 2 + 0) * 8 + i) * 2 + 1] = simulated_stones[base_index + i].position.y;
            temp_result[((sim * 2 + 1) * 8 + i) * 2 + 0] = simulated_stones[base_index + 8 + i].position.x;
            temp_result[((sim * 2 + 1) * 8 + i) * 2 + 1] = simulated_stones[base_index + 8 + i].position.y;
        }
    }

    std::vector<py::ssize_t> shape = {
        static_cast<py::ssize_t>(simulation_count), // シミュレーション回数
        static_cast<py::ssize_t>(2),                // team0とteam1の2チーム
        static_cast<py::ssize_t>(8),                // 各チームのストーン数
        static_cast<py::ssize_t>(2)                 // ストーンの位置を示す座標(x, y)
    };
    // 4次元配列に変換
    py::array_t<double> result(shape);
    std::memcpy(result.mutable_data(), temp_result.data(), total_size * sizeof(double));

    return result;
}

void SimulatorFCV1::ContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse)
{
    auto a_body = contact->GetFixtureA()->GetBody();
    auto b_body = contact->GetFixtureB()->GetBody();

    digitalcurling3::Collision collision;
    collision.a.id = static_cast<int>(a_body->GetUserData().pointer);
    collision.b.id = static_cast<int>(b_body->GetUserData().pointer);

    add_unique_id(instance_->pending_awake, collision.a.id);
    add_unique_id(instance_->pending_awake, collision.b.id);

    add_unique_id(instance_->moved, collision.a.id);
    add_unique_id(instance_->moved, collision.b.id);

    b2WorldManifold world_manifold;
    contact->GetWorldManifold(&world_manifold);

    collision.normal_impulse = impulse->normalImpulses[0];
    collision.tangent_impulse = impulse->tangentImpulses[0];
}

void SimulatorFCV1::ContactListener::add_unique_id(std::vector<int>& list, int id)
{
    if (std::find(list.begin(), list.end(), id) == list.end())
    {
        list.push_back(id);
    }
}


SimulatorFCV1::SimulatorFCV1(std::vector<digitalcurling3::StoneData> const &stones) : stones(stones), world(b2Vec2(0, 0)), contact_listener_(this)
{
    stone_body_def.type = b2_dynamicBody;
    stone_body_def.awake = false;
    stone_body_def.bullet = true;
    stone_body_def.enabled = false;

    b2CircleShape stone_shape;
    stone_shape.m_radius = kStoneRadius;

    b2FixtureDef stone_fixture_def;
    stone_fixture_def.shape = &stone_shape;
    stone_fixture_def.friction = 0.2f;                                        // 適当というかデフォルト値
    stone_fixture_def.restitution = 1.0;                                      // 完全弾性衝突(完全弾性衝突の根拠は無いし多分違う)
    stone_fixture_def.restitutionThreshold = 0.f;                             // 反発閾値。この値より大きい速度(m/s)で衝突すると反発が適用される。
    stone_fixture_def.density = 0.5f / (b2_pi * kStoneRadius * kStoneRadius); // kg/m^2

    for (size_t i = 0; i < kStoneMax; ++i)
    {
        stone_body_def.userData.pointer = static_cast<uintptr_t>(i);
        stone_bodies[i] = world.CreateBody(&stone_body_def);
        stone_bodies[i]->CreateFixture(&stone_fixture_def);
    }
    world.SetContactListener(&contact_listener_);
}

void SimulatorFCV1::is_freeguardzone()
{
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        auto body = stone_bodies[i];
        float dx = body->GetPosition().x;
        float dy = body->GetPosition().y - tee_line;
        float distance_squared = dx * dx + dy * dy;
        if (dy < 0 && distance_squared > house_radius * house_radius && body->GetPosition().y >= min_y)
        {
            in_free_guard_zone.push_back(i);
        }
    }
}

void SimulatorFCV1::change_shot(int shot)
{
    this->shot = shot;
}

void SimulatorFCV1::is_in_playarea()
{
    for (int i : in_free_guard_zone)
    {
        auto body = stone_bodies[i];
        if (body->GetPosition().y > y_upper_limit || body->GetPosition().x > stone_x_upper_limit || body->GetPosition().x < stone_x_lower_limit)
        {
            for (int index : moved)
            {
                auto stone = stones[index];
                stone_bodies[index]->SetTransform(b2Vec2(stone.position.x, stone.position.y), 0.f);
            }
            return;
        }
    }
    return;
}

// ノーティックルール対応用関数
// void SimulatorFCV1::on_center_line()
// {
//     for (size_t i = 0; i < kStoneMax; ++i)
//     {
//         auto stone_body = stone_bodies[i];
//         if (stone_body.IsEnabled() && stone_radius - std::abs(stone_body->GetPosition().x) < 0.0)
//         {
//             on_center_line.push_back(i);
//         }
//     }
// }

// ノーティックルール対応用関数
// void SimulatorFCV1::no_tick_rule()
// {
//     for (int i : on_center_line)
//     {
//         auto stone_body = stone_bodies[i];
//         if (stone_radius - std::abs(stone_body->GetPosition().x) > 0.0)
//         {
//             for (int index : moved)
//             {
//                 auto stone = stones[index];
//                 stone_bodies[index]->SetTransform(b2Vec2(stone.position.x, stone.position.y), 0.f);
//             }
//             break;
//         }
//     }
// }

void SimulatorFCV1::step(float seconds_per_frame)
{
    int step_i = 0;

    // simulate
    // 正しい繰り返し：現在起きている石の集合を毎フレーム再構築する（走査中に is_awake を変更しない）
    while (!is_awake.empty())
    {
        std::vector<int> next_awake;
        next_awake.reserve(is_awake.size());

        for (int index : is_awake)
        {
            b2Vec2 const stone_velocity = stone_bodies[index]->GetLinearVelocity(); // copy
            auto const [normalized_stone_velocity, stone_speed] = normalize(stone_velocity);
            float const angular_velocity = stone_bodies[index]->GetAngularVelocity();

            // ストーンが停止してる場合は next_awake に入れない（＝除外）
            if (stone_speed > EPSILON)
            {
                digitalcurling3::Vector2 stone_position = {stone_bodies[index]->GetPosition().x, stone_bodies[index]->GetPosition().y};
                // ストーンがシート外の場合は初期位置へ戻して除外 ただし、y座標方向については今回だけ無視
                if (stone_position.x > stone_x_upper_limit || stone_x_lower_limit > stone_position.x)
                {
                    auto stone = stones[index];
                    stone_bodies[index]->SetTransform(b2Vec2(0.f, 0.f), 0.f);
                    stone_bodies[index]->SetLinearVelocity(b2Vec2_zero);
                    stone_bodies[index]->SetAngularVelocity(0.f);
                    stone_bodies[index]->SetAwake(false);
                    stone_bodies[index]->SetEnabled(false);
                    // do not add to next_awake
                    continue;
                }

                // 速度の更新
                float const new_stone_speed = stone_speed + longitudinal_acceleration(stone_speed) * seconds_per_frame;
                if (new_stone_speed <= 0.f)
                {
                    stone_bodies[index]->SetLinearVelocity(b2Vec2_zero);
                    stone_bodies[index]->SetAwake(false);
                    // stopped -> do not add to next_awake
                    continue;
                }
                else
                {
                    float const yaw = yaw_rate(stone_speed, angular_velocity) * seconds_per_frame;
                    float const longitudinal_velocity = new_stone_speed * std::cos(yaw);
                    float const transverse_velocity = new_stone_speed * std::sin(yaw);
                    b2Vec2 const &e_longitudinal = normalized_stone_velocity;
                    b2Vec2 const e_transverse = e_longitudinal.Skew();
                    b2Vec2 const new_stone_velocity = longitudinal_velocity * e_longitudinal + transverse_velocity * e_transverse;
                    stone_bodies[index]->SetLinearVelocity(new_stone_velocity);
                    // still awake -> keep for next frame
                    next_awake.push_back(index);
                }
            }
            else
            {
                // stone_speed <= EPSILON -> treat as stopped
                stone_bodies[index]->SetLinearVelocity(b2Vec2_zero);
                stone_bodies[index]->SetAwake(false);
                // do not add to next_awake
            }

            // 角速度の更新（停止判定と分離して処理）
            if (std::abs(angular_velocity) > EPSILON)
            {
                float const angular_accel = angular_acceleration(stone_speed) * seconds_per_frame;
                float new_angular_velocity = 0.f;
                if (std::abs(angular_velocity) <= std::abs(angular_accel))
                {
                    new_angular_velocity = 0.f;
                }
                else
                {
                    new_angular_velocity = angular_velocity + angular_accel * angular_velocity / std::abs(angular_velocity);
                }
                stone_bodies[index]->SetAngularVelocity(new_angular_velocity);
            }
        }

        // 物理ステップを進める
        world.Step(
            seconds_per_frame,
            8,  // velocityIterations
            3); // positionIterations

        if (!pending_awake.empty())
        {
            for (int id : pending_awake)
            {
                if (std::find(next_awake.begin(), next_awake.end(), id) == next_awake.end())
                    next_awake.push_back(id);
            }
            pending_awake.clear();
        }

        // 次フレームの起きている石集合に入れ替え
        is_awake.swap(next_awake);

        step_i += 1;
        if (step_i > 50000) {
            break;
        }
    }
}

void SimulatorFCV1::set_stones()
{
    // update bodies
    int ally_position_size = shot / 2 + 1;
    int opponent_position_size = shot / 2 + 9;
    for (size_t i = 0; i < ally_position_size; ++i)
    {
        auto &stone = stones[i];
        digitalcurling3::Vector2 position = stone.position;
        if (position.x == 0.f && position.y == 0.f)
        {
            stone_bodies[i]->SetEnabled(false);
        }
        else
        {
            stone_bodies[i]->SetEnabled(true);
            stone_bodies[i]->SetAwake(true);
            stone_bodies[i]->SetTransform(b2Vec2(position.x, position.y), 0.f);
        }
    }

    for (size_t i = 8; i < opponent_position_size; ++i)
    {
        auto &stone = stones[i];
        auto position = stone.position;
        if (position.x == 0.f && position.y == 0.f)
        {
            stone_bodies[i]->SetEnabled(false);
        }
        else
        {
            stone_bodies[i]->SetEnabled(true);
            stone_bodies[i]->SetAwake(true);
            stone_bodies[i]->SetTransform(b2Vec2(position.x, position.y), 0.f);
        }
    }

    if (shot < 5)
    {
        is_freeguardzone();
    }
}

void SimulatorFCV1::set_velocity(float velocity_x, float velocity_y, float angular_velocity, unsigned int shot_per_team, unsigned int team_id, int id)
{
    shot_id = id;
    int index = shot_per_team + team_id * 8;

    stone_bodies[index]->SetLinearVelocity(b2Vec2(velocity_x, velocity_y));
    stone_bodies[index]->SetAngularVelocity(angular_velocity);
    stone_bodies[index]->SetEnabled(true);
    stone_bodies[index]->SetAwake(true);
    stone_bodies[index]->SetTransform(b2Vec2(0.0, 0.0), 0.f);
    is_awake.push_back(index);
    moved.push_back(index);
}

digitalcurling3::StoneDataWithID SimulatorFCV1::get_stones()
{
    digitalcurling3::StoneDataWithID stones_data;
    for (b2Body *body : stone_bodies)
    {
        b2Vec2 position = body->GetPosition();
        // ここも、y方向のみ
        if (position.x > stone_x_upper_limit || position.x < stone_x_lower_limit)
        {
            body->SetTransform(b2Vec2(0.f, 0.f), 0.f);
        }
        b2Vec2 after_position = body->GetPosition();
        stones_data.stones.push_back({digitalcurling3::Vector2(after_position.x, after_position.y)});
    }
    stones_data.id = shot_id;
    return stones_data;
}


StoneSimulator::StoneSimulator() : storage(), total_shot() {
    this->x_velocities.reserve(10000);
    this->y_velocities.reserve(10000);
    this->angular_velocities.reserve(10000);
    storage.reserve(16);
    simulated_stones.reserve(10000);

    num_threads = read_configfile("config.json")["thread_num"];
    omp_set_num_threads(num_threads);
    simulators.resize(num_threads);
    local_simulated_stones.resize(num_threads, std::vector<digitalcurling3::StoneDataWithID>());
    #pragma omp parallel num_threads(num_threads)
    {
        // Empty block. No operations are performed in the threads.
    }
}


    /// \brief Function to call from python
    /// \param[in] team0_positions The positions of the stones for team 0
    /// \param[in] team1_positions The positions of the stones for team 1
    /// \param[in] shot The number of shots
    /// \param[in] x_velocities The x component of the velocity of the stone to be thrown
    /// \param[in] y_velocities The y component of the velocity of the stone to be thrown
    /// \param[in] angular_velocities 1 -> cw, -1 -> ccw
    /// \returns The positions of the stones after the simulations
py::array_t<double> StoneSimulator::simulator(py::array_t<double> team0_stone_positions, py::array_t<double> team1_stone_positions, int total_shot, py::array_t<double> x_velocities, py::array_t<double> y_velocities, py::array_t<int> angular_velocities, unsigned int team_id, unsigned int shot_per_team, unsigned int hummer_team)
{
    this->total_shot = total_shot;
    this->shot_per_team = shot_per_team;
    this->team_id = team_id;
    x_velocities_length = len(x_velocities);
    this->x_velocities.clear();
    this->y_velocities.clear();
    this->angular_velocities.clear();
    const py::buffer_info &team0_stone_positions_buf = team0_stone_positions.request();
    const py::buffer_info &team1_stone_positions_buf = team1_stone_positions.request();
    simulated_stones.clear();
    storage.clear();
    for (int i = 0; i < num_threads; i++)
    {
        local_simulated_stones[i].clear();
    }
    for (size_t i = 0; i < x_velocities_length; ++i)
    {
        this->x_velocities.push_back(x_velocities.at(i));
        this->y_velocities.push_back(y_velocities.at(i));
        this->angular_velocities.push_back(angular_velocities.at(i) * cw);
    }
    simulated_stones.resize(x_velocities_length);

    if (hummer_team == 1)
    {
        for (int i = 0; i < team0_stone_positions_buf.shape[0]; i++)
        {
            storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team0_stone_positions.data(i, 0), *team0_stone_positions.data(i, 1))));
            // storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team1_stone_positions.data(i, 0), *team1_stone_positions.data(i, 1))));
        }
        for (int i = 0; i < team1_stone_positions_buf.shape[0]; i++)
        {
            storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team1_stone_positions.data(i, 0), *team1_stone_positions.data(i, 1))));
        }
    }
    else if (hummer_team == 0)
    {
        for (int i = 0; i < team1_stone_positions_buf.shape[0]; i++)
        {
            storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team1_stone_positions.data(i, 0), *team1_stone_positions.data(i, 1))));
            // storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team0_stone_positions.data(i, 0), *team0_stone_positions.data(i, 1))));
        }
        for (int i = 0; i < team0_stone_positions_buf.shape[0]; i++)
        {
            storage.push_back(digitalcurling3::StoneData(digitalcurling3::Vector2(*team0_stone_positions.data(i, 0), *team0_stone_positions.data(i, 1))));
        }
    }
    else
    {
        std::cerr << "Invalid hummer_team value: " << hummer_team << ". It should be either 0 or 1." << std::endl;
    }

    for (int i = 0; i < num_threads; i++)
    {
        if (simulators[i] != nullptr) {     // ← 既存を解放
            delete simulators[i];
            simulators[i] = nullptr;
        }
        simulators[i] = new SimulatorFCV1(storage);
        simulators[i]->change_shot(this->total_shot);
    }

    simulated_stones.clear();
    simulated_stones.resize(static_cast<size_t>(x_velocities_length));

    #pragma omp parallel for num_threads(num_threads) schedule(static, 1)
    for (int i = 0; i < x_velocities_length; ++i)
    {
        int thread_id = omp_get_thread_num();
        simulators[thread_id]->set_stones();
        simulators[thread_id]->set_velocity(this->x_velocities[i],
                                            this->y_velocities[i],
                                            this->angular_velocities[i],
                                            this->shot_per_team,
                                            this->team_id,
                                            i);
        simulators[thread_id]->step(0.001f);
        simulators[thread_id]->is_in_playarea();

        // move で代入（get_stones() の戻り値をムーブする）
        simulated_stones[i] = std::move(simulators[thread_id]->get_stones());
    }

    // flatten into state_values (ここで state_values を作る)
    state_values.clear();
    for (const digitalcurling3::StoneDataWithID &stone_with_id : simulated_stones)
    {
        for (const digitalcurling3::StoneData &stone : stone_with_id.stones)
        {
            state_values.push_back(stone);
        }
    }

    // sanity check
    size_t expected_total = static_cast<size_t>(x_velocities_length) * stones_per_simulation;
    if (state_values.size() != expected_total)
    {
        std::cerr << "STATE_VALUES SIZE MISMATCH: got=" << state_values.size()
                  << " expected=" << expected_total << std::endl;
    }

    result = convert_stonedata(state_values, x_velocities_length);
    return result;
}



// main関数

PYBIND11_MODULE(simulator, m)
{
    py::class_<StoneSimulator>(m, "StoneSimulator")
        .def(py::init<>())
        .def("simulator", &StoneSimulator::simulator);
}