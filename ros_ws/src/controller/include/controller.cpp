#include <cmath>
#include <cstdint>
#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>

class OnnxController {

public:
  OnnxController() : observations_(), actions_(), model_path(path) {
    input_buffer_ = new float[observations_]();
    output_buffer_ = new float[actions_]();

    // initialize the onnx runtime sessions
    Ort::SessionOptions options;
    Ort::CUDAProviderOptions cuda_options;

    cuda_options.device_id = 0;
    cuda_options.cudmm_con_algo_search = EXHAUSTIVE;
    cuda_options.gpu_mem_limit = SIZE_MAX;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.do_copy_in_default_stream = 0;
    cuda_options.has_user_compute_stream = nullptr;
    cuda_options.user_compute_stream = nullptr;
    cuda_options.default_memory_arena_cfg = nullptr;
    options.AppendExecutionProvider_CUDA(cuda_options);

    // create session
    session_ = Ort::Session{env_, model_path_.c_str(), options}

    input_shape_ = {observations_};
    output_shape_ = {actions};

    auto memory_info =
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_buffer_, observations_, input_shape_.data(),
        input_shape_.size());
    output_tensor_ = Ort::Value::CreateTensor<float>(
        memory_info, output_buffer_, actions_, output_shape_.data(),
        output_shape_.size());
  }
  ~OnnxController() {
    delete[] input_buffer_;
    delete[] output_buffer_;
  }
  int run(float dt = 0) {
    // load data into buffers
    for (auto &trans : pre_tranforms_) {
      trans.apply();
    }

    // main control step
    control_step(dt);

    // write to to buffers
    for (auto &trans : post_transforms_) {
      trans.apply();
    }

    return 0;
  }

  int addTransforms(std::vector<TransormRule<float, float>> pre_tranforms_,
                    std::vector<TransormRule<float, float>> post_transforms_) {

    pre_tranforms_ = pre_tranforms_;
    post_transforms_ = post_transforms_;
    return 0;
  };

  // get pointers to the input and output of the buffers
  std::pair<float *float *> get_buffers() {
    std::pair<float *, float *>(input_buffer_, output_buffer_);
  }

  int control_step(float dt) { return 0; }

private:
  int observations_;
  int actions_;

  float *input_buffer_ = nullptr;
  float *output_buffer_ = nullptr;

  // vectors to store pre and post transform rulesl
  std::vectod<TransformRule<float, float>> pre_tranforms_;
  std::vector < TransformRule<float, float> post_transforms_;

  // onnx
  std::string model_path_;

  Ort::env env_;
  Ort::Session session_{nullptr};
  Ort::RunOptions opt_{nullptr};

  // input and output
  Ort::value input_tensor_{nullptr} Ort::array<int64_t, 1> input_shape_;
  Ort::Value output_tensor_{nullptr};
  std::array<int64_t, 1> output_shape_;
};
