#include "simulate.h"

namespace plt = matplot;

SDL_AudioSpec flightSpec, restSpec;
Uint32 flightLength, restLength;
Uint8* flightBuffer, * restBuffer;
SDL_AudioDeviceID audioDevice;

enum class SoundType { REST, FLIGHT };

struct AudioData {
    Uint8* pos;
    Uint32 length;
    Uint8* buffer;
    Uint32 bufferLength;
    SoundType soundType;
} audioData;

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}


void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void plotData(const std::vector<float>& x_history, const std::vector<float>& y_history, const std::vector<float>& theta_history) {
    plt::figure();
    plt::subplot(3, 1, 1);
    plt::plot(x_history);
    plt::title("X Position over Time");

    plt::subplot(3, 1, 2);
    plt::plot(y_history);
    plt::title("Y Position over Time");

    plt::subplot(3, 1, 3);
    plt::plot(theta_history);
    plt::title("Theta over Time");

    plt::show();
}

void playSound(SoundType type) {
    audioData.buffer = (type == SoundType::REST) ? restBuffer : flightBuffer;
    audioData.bufferLength = (type == SoundType::REST) ? restLength : flightLength;
    audioData.pos = audioData.buffer;
    audioData.length = audioData.bufferLength;
    audioData.soundType = type;
    SDL_PauseAudioDevice(audioDevice, 0);
}

void audioCallback(void* userdata, Uint8* stream, int len) {
    AudioData* audio = static_cast<AudioData*>(userdata);
    if (audio->length == 0) return;

    len = (len > audio->length) ? audio->length : len;
    SDL_memcpy(stream, audio->pos, len);
    audio->pos += len;
    audio->length -= len;

    if (audio->length <= 0) {
        audio->pos = audio->buffer;
        audio->length = audio->bufferLength;
    }
}


int main(int argc, char* args[]) {
    std::shared_ptr<SDL_Window> gWindow;
    std::shared_ptr<SDL_Renderer> gRenderer;
    const int SCREEN_WIDTH = 1280, SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    quadrotor.SetGoal(goal_state);

    const float dt = 0.01f;
    Eigen::MatrixXf K = LQR(quadrotor, dt);

    std::vector<float> x_history, y_history, theta_history;
    int target_x = -1, target_y = -1;
    bool target_reached = false;


    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0) {
        SDL_Event e;
        bool quit = false;


        playSound(SoundType::REST);

        while (!quit) {
            while (SDL_PollEvent(&e) != 0) {
                if (e.type == SDL_QUIT) {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    SDL_GetMouseState(&target_x, &target_y);
                    std::cout << "Clicked at position: (" << target_x << ", " << target_y << ")" << std::endl;

                    float quadrotor_x = (float)target_x - (float)SCREEN_WIDTH / 2.0f;
                    float quadrotor_y = (float)SCREEN_HEIGHT / 2.0f - (float)target_y;


                    Eigen::VectorXf new_goal_state = Eigen::VectorXf::Zero(6);
                    new_goal_state << quadrotor_x, quadrotor_y, quadrotor.GetState()[2], 0, 0, 0;
                    quadrotor.SetGoal(new_goal_state);
                    target_reached = false;

                    playSound(SoundType::FLIGHT);
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                    plotData(x_history, y_history, theta_history);
                }
            }

            SDL_Delay(static_cast<int>(dt * 10));
            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            if (!target_reached && target_x >= 0 && target_y >= 0) {
                Eigen::VectorXf state = quadrotor.GetState();
                float dx = target_x - (state[0] + SCREEN_WIDTH / 2.0f);
                float dy = target_y - (SCREEN_HEIGHT / 2.0f - state[1]);
                if (std::sqrt(dx * dx + dy * dy) < 100.0f) {
                    target_reached = true;
                    playSound(SoundType::REST);
                }
                else {
                    filledCircleRGBA(gRenderer.get(), target_x, target_y, 5, 0xFF, 0x00, 0x00, 0xFF);
                }
            }

            quadrotor_visualizer.render(gRenderer);
            SDL_RenderPresent(gRenderer.get());

            control(quadrotor, K);
            quadrotor.Update(dt);


            Eigen::VectorXf state = quadrotor.GetState();
            x_history.push_back(state[0]);
            y_history.push_back(state[1]);
            theta_history.push_back(state[2]);
        }
    }

    SDL_CloseAudioDevice(audioDevice);
    SDL_FreeWAV(flightBuffer);
    SDL_FreeWAV(restBuffer);
    SDL_Quit();

    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, int screenWidth, int screenHeight) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }

    gWindow.reset(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
    gRenderer.reset(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);

    if (SDL_LoadWAV("C:\\Users\\bagro\\OneDrive\\Pulpit\\tp_projekt_4\\sound\\fly.wav", &flightSpec, &flightBuffer, &flightLength) == NULL ||
        SDL_LoadWAV("C:\\Users\\bagro\\OneDrive\\Pulpit\\tp_projekt_4\\sound\\stay.wav", &restSpec, &restBuffer, &restLength) == NULL) {
        std::cout << "SDL_ERROR: Nie mo¿na za³adowaæ pliku WAV " << SDL_GetError() << std::endl;
        return -1;
    }

    audioData = { restBuffer, restLength, restBuffer, restLength, SoundType::REST };
    restSpec.callback = audioCallback;
    restSpec.userdata = &audioData;

    audioDevice = SDL_OpenAudioDevice(NULL, 0, &restSpec, NULL, 0);
    if (audioDevice == 0) {
        std::cout << "SDL_ERROR: Nie mo¿na otworzyæ urz¹dzenia audio " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_PauseAudioDevice(audioDevice, 0);
    return 0;
}