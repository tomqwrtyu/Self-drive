CAN 10a   JLL, 2021.4.21-4.27, 5.27-6.24, 30
Code Overview: From Neural Net to CAN by Steering
--------------------------------------------------

---------- Step 1: Model Driving
----- Input 1: sm["frame"].capnp.getFrame().getFrameId()
----- Input 2: frame_init.transform_init().loadyuv_init(width, height)
----- Load Model: model_init.DefaultRunModel=SNPEModel(supercombo.dlc).read_file()
----- Execute Model: model_eval_frame.RunModel=SNPEModel.SNPE.execute()
----- Output: "model", ModelData: pm.send(1, 2).send(1, 2, 3).PubSocket.ZMQPubSocket.send(1, 2).zmq_send(1, 2, 3, 4)
----- Keywords: ModelData, ModelState, yuv, cereal::Event, capnp, zmq

/OP081/selfdrive/modeld/modeld
/OP081/selfdrive/modeld/modeld.cc
/OP081/selfdrive/modeld/models/driving.h
/OP081/cereal/messaging/messaging.hpp
/OP081/selfdrive/modeld/models/driving.cc
/OP081/selfdrive/modeld/models/commonmodel.h
/OP081/selfdrive/modeld/models/commonmodel.cc
/OP081/selfdrive/modeld/runners/run.h
/OP081/selfdrive/modeld/runners/runmodel.h
/OP081/selfdrive/modeld/runners/snpemodel.h
/OP081/selfdrive/modeld/runners/snpemodel.cc
/OP081/phonelibs/snpe/include/SNPE/SNPE.hpp
/OP081/selfdrive/common/visionbuf_ion.c
/OP081/selfdrive/common/visionipc.h
/OP081/selfdrive/common/visionbuf.h
/OP081/selfdrive/common/modeldata.h
/OP081/cereal/messaging/socketmaster.cc
/OP081/cereal/messaging/messaging.cc
/OP081/cereal/messaging/impl_zmq.hpp
/OP081/cereal/messaging/impl_zmq.cc
/OP081/cereal/gen/cpp/log.capnp.h
/OP081/cereal/log.capnp
/OP081/cereal/services.h
--------------------------------------------------

/OP081/selfdrive/modeld/modeld
  #!/bin/sh
  if [ -d /system ]; then
      if [ -f /TICI ]; then # QCOM2
          export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:/data/pythonpath/phonelibs/snpe/larch64:$LD_LIBRARY_PATH"
      else # QCOM
        export LD_LIBRARY_PATH="/data/pythonpath/phonelibs/snpe/aarch64/:$LD_LIBRARY_PATH"
      fi
  else # PC
      export LD_LIBRARY_PATH="$HOME/openpilot/phonelibs/snpe/x86_64-linux-clang:/openpilot/phonelibs/snpe/x86_64:$HOME/openpilot/phonelibs/snpe/x86_64:$LD_LIBRARY_PATH"
  fi
  exec ./_modeld

--- #!/bin/sh executes the script using the Bourne shell with path /bin/sh
    sh is an executable shell which is an interactive interface that
      allows users to execute other commands and utilities in Linux.
    "#!" is an operator called shebang which directs the script to
      the interpreter location. # is a comment (not excuted)
--- if [ -d /system ]; then
      [[ ]]/[] is a bash/dash conditional operator
      -d file	True if file is a directory.
      -f file	True if file exists and is a regular file.
    Dash (Debian Almquist shell) is a Linux shell which is much smaller than
      bash (GNU Bourne-Again Shell) but still aiming at POSIX-compliancy.
--- /selfdrive/modeld/_modeld _modeld is excutable.

/OP081/selfdrive/modeld/modeld.cc
  #include "common/visionipc.h" // VisionStream, VisionStreamBufs
  #include "common/visionbuf.h" // VisionBuf
  #include "models/driving.h" // ModelDataRaw, ModelState
  #include "messaging.hpp" // PubMaster, SubMaster
  bool run_model;
  mat3 cur_transform;
  void* live_thread(void *arg) {
    set_thread_name("live");
    SubMaster sm({"liveCalibration"});
    Eigen::Matrix<float, 3, 3> ground_from_medmodel_frame;
    ground_from_medmodel_frame <<
      0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
    Eigen::Matrix<float, 3, 3> fcam_intrinsics;
    fcam_intrinsics <<
      910.0, 0.0, 582.0,
----- Input 2: yuv_transform
    mat3 yuv_transform = transform_scale_buffer((mat3){{
      1.0, 0.0, 0.0,
    while (!do_exit) {
      if (sm.update(100) > 0){
        Eigen::Matrix<float, 3, 4> extrinsic_matrix_eigen;
        for (int i = 0; i < 4*3; i++){
          extrinsic_matrix_eigen(i / 4, i % 4) = extrinsic_matrix[i];  }
        auto camera_frame_from_road_frame = fcam_intrinsics * extrinsic_matrix_eigen;
        Eigen::Matrix<float, 3, 3> camera_frame_from_ground;
        camera_frame_from_ground.col(0) = camera_frame_from_road_frame.col(0);
        auto warp_matrix = camera_frame_from_ground * ground_from_medmodel_frame;
        mat3 transform = {};
        for (int i=0; i<3*3; i++) {
          transform.v[i] = warp_matrix(i / 3, i % 3);  }
----- Input 2: yuv_transform.model_transform
        mat3 model_transform = matmul3(yuv_transform, transform);
----- Input 2: yuv_transform.model_transform.cur_transform
        cur_transform = model_transform;
        run_model = true;
  int main(int argc, char **argv) {
    // start calibration thread
    err = pthread_create(, , live_thread, );
    assert(err == 0);
    // messaging
----- Input 1: sm({"frame"})
----- Output: pm({"model"})
    PubMaster pm({"modelV2", "model", });
    SubMaster sm({, "frame"});
    ModelState model;
----- Load Model: model_init(&model)
    model_init(&model, device_id, context);
    LOGW("models loaded, modeld starting");
    VisionStream stream;
    while (!do_exit) { // while 1
      VisionStreamBufs buf_info;
      err = visionstream_init(&stream, VISION_STREAM_YUV, true, &buf_info);
----- Input 2: yuv_ion.visionbuf_allocate_cl
      VisionBuf yuv_ion = visionbuf_allocate_cl(buf_info.buf_len, device_id, context);
      uint32_t frame_id = 0, last_vipc_frame_id = 0;
      while (!do_exit) { // while 2
        VIPCBuf *buf;
        VIPCBufExtra extra;
        buf = visionstream_get(&stream, &extra);
----- Input 2: yuv_transform.model_transform.cur_transform.model_transform
        mat3 model_transform = cur_transform;
        const bool run_model_this_iter = run_model;
        if (sm.update(0) > 0){
----- Input 1: sm["frame"].getFrame().getFrameId()
          frame_id = sm["frame"].getFrame().getFrameId();}
        if (run_model_this_iter) {
          memcpy(yuv_ion.addr, buf->addr, buf_info.buf_len);
----- Execute Model: model_eval_frame(&model)
          ModelDataRaw model_buf =  model_eval_frame(&model, yuv_ion.buf_cl, buf_info.width, buf_info.height,
                                    model_transform, vec_desire);
          const float *raw_pred_ptr = send_raw_pred ? &model.output[0] : nullptr;
----- Output: model_publish(pm, ModelDataRaw)
          model_publish(pm, frame_id, model_buf, raw_pred_ptr,);
          posenet_publish(pm, extra.frame_id, model_buf,);
          LOGD("model process: %.2fms, from last %.2fms, vipc_frame_id %zu, frame_id, %zu, frame_drop %.3f", mt2-mt1, mt1-last, extra.frame_id, frame_id, frame_drop_ratio);}
      } // while 2 end
      visionbuf_free(&yuv_ion);
      visionstream_destroy(&stream);}
    } // while 1 end
    model_free(&model);}

--- Eigen::Matrix: The three mandatory template parameters of Matrix are
       Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    Eigen offers matrix/vector arithmetic operations either through overloads of
      common C++ arithmetic operators such as +, -, *, or through special methods
      such as dot(), cross(), etc.
    In Eigen, all matrices and vectors are objects of the Matrix template class.
    https://eigen.tuxfamily.org/dox/GettingStarted.html
    https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
    https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
--- return ( m > n ? m : n ); == if( m > n ) return m; else return n;
    The ? and : are part of conditional operators syntax.
    The ? indicates that the left expression should be a boolean evaluation true or false.
    The : delineates the true and false results.

/OP081/selfdrive/modeld/models/driving.h
  #include "common/modeldata.h"
  #include "commonmodel.h"
  #include "runners/run.h"
  #include "messaging.hpp"
  constexpr int DESIRE_LEN = 8;
  constexpr int TRAFFIC_CONVENTION_LEN = 2;
  constexpr int MODEL_FREQ = 20;
----- Output: model_publish(pm, ModelDataRaw)
  struct ModelDataRaw {
      float *plan;
      float *lane_lines;
      float *lane_lines_prob;
      float *road_edges;
      float *lead;
      float *lead_prob;
      float *desire_state;
      float *meta;
      float *desire_pred;
      float *pose;};
  typedef struct ModelState {
    ModelFrame frame;
    std::unique_ptr<float[]> output;
    std::unique_ptr<float[]> input_frames;
    std::unique_ptr<RunModel> m;
    float prev_desire[DESIRE_LEN] = {};
    float pulse_desire[DESIRE_LEN] = {};
  } ModelState;
----- Load Model: model_init(ModelState)
  void model_init(ModelState* s, cl_device_id device_id, cl_context context);
  ModelDataRaw model_eval_frame(ModelState* s, cl_mem yuv_cl, int width, int height,
                                const mat3 &transform, float *desire_in);
  void model_free(ModelState* s);
  void poly_fit(float *in_pts, float *in_stds, float *out);
  void model_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t frame_id, float frame_drop,
  void posenet_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,

--- typedef struct ModelState: struct and typedef are two very different things.
    The struct keyword is used to define (create), or to refer to, a structure type.
    A typedef, in spite of the name, does not define a new type;
      it merely creates a new name for an existing type like "typedef int my_int;".
    It allows to write "void f(ModelState m);" otherwise to "void f(struct ModelDataRaw mdr);"
    https://www.cplusplus.com/forum/beginner/227625/
    https://stackoverflow.com/questions/1675351/typedef-struct-vs-struct-definitions

/OP081/cereal/messaging/messaging.hpp
  #include <capnp/serialize.h>
  #include "../gen/cpp/log.capnp.h"
--- for c++ code
  class Message {
    virtual size_t getSize() = 0;
    virtual char * getData() = 0;
  class SubSocket {
  class Poller {
  class SubMaster {
    SubMaster(const std::initializer_list<const char *> &service_list,
              const char *address = nullptr, const std::initializer_list<const char *> &ignore_alive = {});
    int update(int timeout = 1000);
    uint64_t frame = 0;
    uint64_t rcv_frame(const char *name) const;
----- Input 1: sm["frame"].capnp.getFrame().getFrameId()
    cereal::Event::Reader &operator[](const char *name); // cereal::Event::Reader is in log.capnp.h
  class MessageBuilder : public capnp::MallocMessageBuilder {
    MessageBuilder() = default;
    cereal::Event::Builder initEvent(bool valid = true) {
      cereal::Event::Builder event = initRoot<cereal::Event>();
      struct timespec t;
      clock_gettime(CLOCK_BOOTTIME, &t);
      uint64_t current_time = t.tv_sec * 1000000000ULL + t.tv_nsec;
      event.setLogMonoTime(current_time);
      event.setValid(valid);
      return event;
    kj::ArrayPtr<capnp::byte> toBytes() {
      heapArray_ = capnp::messageToFlatArray(*this);
      return heapArray_.asBytes();
  class PubMaster {
    PubMaster(const std::initializer_list<const char *> &service_list);
----- Output: model_publish(pm, ModelDataRaw).pm.send(1, 2)
    int send(const char *name, MessageBuilder &msg);
----- Output: model_publish(pm, ModelDataRaw).pm.send(1, 2).send(1, 2, 3).PubSocket.send(1, 2)
    inline int send(const char *name, capnp::byte *data, size_t size)
      { return sockets_.at(name)->send((char *)data, size); }
    std::map<std::string, PubSocket *> sockets_;
  class PubSocket {
    virtual int connect(Context *context, std::string endpoint) = 0;
    virtual int sendMessage(Message *message) = 0;
----- Output: model_publish(pm, ModelDataRaw).pm.send(1, 2).send(1, 2, 3).PubSocket.send(1, 2)
    virtual int send(char *data, size_t size) = 0; // send() defined in impl_zmq.cc
    static PubSocket * create();
    static PubSocket * create(Context * context, std::string endpoint);

--- virtual size_t getSize() = 0: Why is a C++ pure virtual function initialized by 0?
    It’s just a syntax, saying that “the function is pure virtual”.
    It is not defined in base class and should be defined in derived class.
    A virtual function can be defined in base and derived classes.
    https://www.geeksforgeeks.org/difference-between-virtual-function-and-pure-virtual-function-in-c/
--- A virtual function is a member function which is declared within
      a base class and is re-defined (overriden) by a derived class.
    https://www.geeksforgeeks.org/virtual-function-cpp/
--- &operator[](const char *name) => sm["frame"]: &operator[](T) is an
      overloading [] operator to access elements (of type T) in array style.
    The subscript operator [] is normally used to access array elements.
    https://www.geeksforgeeks.org/overloading-subscript-or-array-index-operator-in-c/
--- MessageBuilder() is declared in capnp/message.h or serialize.h?
--- MessageBuilder: cereal::Event::Builder
    To create a new message, most users use capnp::MallocMessageBuilder.
      First allocate a MessageBuilder buffer: message for your message to go in.
      Then allocate an actual object: addressBook from your schema <AddressBook>.
        ::capnp::MallocMessageBuilder message;
        AddressBook::Builder addressBook = message.initRoot<AddressBook>();
      https://capnproto.org/cxx.html
--- kj::Array is in selfdrive/loggerd/logger.h  #include <kj/array.h>
--- capnp: Cap’n Proto is built on top of a basic utility library called KJ.
    Cap’n Proto is a data serialization format and Remote Procedure Call framework
      for exchanging data between computer programs.
    The Cap’n Proto C++ runtime implementation provides an easy-to-use interface
      for manipulating messages backed by fast pointer arithmetic.
    https://capnproto.org/cxx.html
    https://capnproto.org/cxx.html#kj-library
    https://github.com/capnproto/capnproto/tree/master/c%2B%2B/src/kj
    https://capnproto.org/encoding.html
--- std::map<K,V> is an associative container that store elements in key-value pair.
      std::map<std::string, int> mymap = {{ "alpha", 0 }, { "beta", 0 } };
      mymap.at("alpha") = 10;
    std::map::at returns a reference to the mapped value of the element identified with key k.
    std::map<std::string, PubSocket *> sockets_ => sockets_.at(name) == PubSocket
    https://en.cppreference.com/w/cpp/container/map

/OP081/selfdrive/modeld/models/driving.cc
  #include <eigen3/Eigen/Dense>
  #include "driving.h"
  constexpr int MODEL_WIDTH = 512;
  constexpr int MODEL_HEIGHT = 256;
  constexpr int MODEL_FRAME_SIZE = MODEL_WIDTH * MODEL_HEIGHT * 3 / 2; // 2???
  constexpr int MIN_VALID_LEN = 10;
  constexpr int TRAJECTORY_TIME = 10;
  constexpr float TRAJECTORY_DISTANCE = 192.0;
  constexpr int TEMPORAL_SIZE = 512;
  constexpr int OUTPUT_SIZE =  POSE_IDX + POSE_SIZE;
----- Input 2: frame_init(&s->frame,,,,context)
----- Load Model: model_init.DefaultRunModel(path/supercombo.dlc, &out, size, runtime)
  void model_init(ModelState* s, cl_device_id device_id, cl_context context, int temporal) {
    frame_init(&s->frame, MODEL_WIDTH, MODEL_HEIGHT, device_id, context);
    s->input_frames = std::make_unique<float[]>(MODEL_FRAME_SIZE * 2);
    const int output_size = OUTPUT_SIZE + TEMPORAL_SIZE;
    s->output = std::make_unique<float[]>(output_size);
    s->m = std::make_unique<DefaultRunModel>("../../models/supercombo.dlc", &s->output[0], output_size, USE_GPU_RUNTIME);
----- Execute Model: model_eval_frame.s->m->execute(&, )
  ModelDataRaw model_eval_frame(ModelState* s, cl_mem yuv_cl, int width, int height,
                           const mat3 &transform, float *desire_in) {
    s->m->execute(&s->input_frames[0], MODEL_FRAME_SIZE*2);
    ModelDataRaw net_outputs;
    net_outputs.plan = &s->output[PLAN_IDX];
    net_outputs.lane_lines = &s->output[LL_IDX];
    net_outputs.lane_lines_prob = &s->output[LL_PROB_IDX];
    net_outputs.road_edges = &s->output[RE_IDX];
    net_outputs.lead = &s->output[LEAD_IDX];
    net_outputs.lead_prob = &s->output[LEAD_PROB_IDX];
    net_outputs.meta = &s->output[DESIRE_STATE_IDX];
    net_outputs.pose = &s->output[POSE_IDX];
    return net_outputs;
  void poly_fit(float *in_pts, float *in_stds, float *out, int valid_len) {
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1> > pts(in_pts, valid_len);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1> > std(in_stds, valid_len);
    Eigen::Map<Eigen::Matrix<float, POLYFIT_DEGREE - 1, 1> > p(out, POLYFIT_DEGREE - 1);
    float y0 = pts[0];
    pts = pts.array() - y0;
    // Build Least Squares equations
    Eigen::Matrix<float, Eigen::Dynamic, POLYFIT_DEGREE - 1> lhs = vander.topRows(valid_len).array().colwise() / std.array();
    Eigen::Matrix<float, Eigen::Dynamic, 1> rhs = pts.array() / std.array();
    // Improve numerical stability
    Eigen::Matrix<float, POLYFIT_DEGREE - 1, 1> scale = 1. / (lhs.array()*lhs.array()).sqrt().colwise().sum();
    lhs = lhs * scale.asDiagonal();
    // Solve inplace
    p = lhs.colPivHouseholderQr().solve(rhs);
    // Apply scale to output
    p = p.transpose() * scale.asDiagonal();
    out[3] = y0;
  void fill_path(cereal::ModelData::PathData::Builder path, const float *data, const float prob,
                 float valid_len, int valid_len_idx, int ll_idx) {
    float points[TRAJECTORY_SIZE] = {};
    float stds[TRAJECTORY_SIZE] = {};
    float poly[POLYFIT_DEGREE] = {};
    for (int i=0; i<TRAJECTORY_SIZE; i++) {
      // negative sign because mpc has left positive
      if (ll_idx == 0) {
        points[i] = -data[30 * i + 16];
        stds[i] = exp(data[30 * (33 + i) + 16]);
      } else {
        points[i] = -data[2 * 33 * ll_idx + 2 * i];
        stds[i] = exp(data[2 * 33 * (4 + ll_idx) + 2 * i]);
    const float std = stds[0];
    poly_fit(points, stds, poly, valid_len_idx);
    path.setPoly(poly);
    path.setProb(prob);
    path.setStd(std);
    path.setValidLen(valid_len);
  void fill_model(cereal::ModelData::Builder &framed, const ModelDataRaw &net_outputs) {
    // Find the distribution that corresponds to the most probable plan
    const float *best_plan = get_plan_data(net_outputs.plan);
    // x pos at 10s is a good valid_len
    float valid_len = 0;
    for (int i=1; i<TRAJECTORY_SIZE; i++) {
      if (const float len = best_plan[30*i]; len >= valid_len){
        valid_len = len;
    // clamp to 10 and MODEL_PATH_DISTANCE
    valid_len = fmin(MODEL_PATH_DISTANCE, fmax(MIN_VALID_LEN, valid_len));
    int valid_len_idx = 0;
    for (int i=1; i<TRAJECTORY_SIZE; i++) {
      if (valid_len >= X_IDXS[valid_len_idx]){
        valid_len_idx = i;
    fill_path(framed.initPath(), best_plan, 1.0, valid_len, valid_len_idx, 0);
    fill_path(framed.initLeftLane(), net_outputs.lane_lines, sigmoid(net_outputs.lane_lines_prob[1]), valid_len, valid_len_idx, 1);
    fill_path(framed.initRightLane(), net_outputs.lane_lines, sigmoid(net_outputs.lane_lines_prob[2]), valid_len, valid_len_idx, 2);
----- Output: model_publish(ModelDataRaw).pm.send(pub_name=="model", msg==framed==ModelData) defined in socketmaster.cc
  void model_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t frame_id, float frame_drop,
                     const ModelDataRaw &net_outputs, const float *raw_pred, uint64_t timestamp_eof,
                     float model_execution_time) {
    const uint32_t frame_age = (frame_id > vipc_frame_id) ? (frame_id - vipc_frame_id) : 0;
    auto do_publish = [&](auto init_model_func, const char *pub_name) {
      MessageBuilder msg;
      auto framed = (msg.initEvent().*(init_model_func))();
      if (send_raw_pred) {
        framed.setRawPred(kj::arrayPtr((const uint8_t *)raw_pred, (OUTPUT_SIZE + TEMPORAL_SIZE) * sizeof(float)));
      fill_model(framed, net_outputs);
      pm.send(pub_name, msg);};
    do_publish(&cereal::Event::Builder::initModel, "model");

--- constexpr int MODEL_WIDTH: constexpr specifies that the value of an object or
      a function can be evaluated at compile time. The idea is to spend time in
      compilation and save time at run time to improve performance.
    In inline functions, expressions are always evaluated at run time.
      constexpr is different, here expressions are evaluated at compile time.
    https://www.geeksforgeeks.org/understanding-constexper-specifier-in-c/
--- std::unique_ptr: Pointers are used for accessing heap memory.
    The problem with heap memory is that when you don’t need it you must deallocate itself.
    Smart pointers (like unique_ptr) automatically manage memory and deallocate the object
      when they are not in use or out of memory scope. unique_ptr also improves security.
    https://www.geeksforgeeks.org/memory-layout-of-c-program/
    https://www.geeksforgeeks.org/auto_ptr-unique_ptr-shared_ptr-weak_ptr-2/
--- std::make_unique<T>(args) constructs and returns an object of type T(args)
      and wraps it in a std::unique_ptr.
    https://www.enseignement.polytechnique.fr/informatique/INF478/docs/Cpp/en/cpp/memory/unique_ptr/make_unique.html
--- s->m = std::make_unique<DefaultRunModel>(path/supercombo.dlc, &, , )
      => s->m == DefaultRunModel(path/supercombo.dlc, &, , )
--- Eigen::Map<V> A(data, 4) maps an array A of m > 4 data elements to a vector V
       having the first 4 coefficients (elements).
    https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html
--- auto do_publish: C++ auto: auto y = sin(1.3); == double y = sin(1.3);
    auto a = 1 + 2;
    auto c0 = a; // type of c0 is int, a placeholder holding a copy of a, i.e., c0 is a.
    https://en.cppreference.com/w/cpp/language/auto
--- [&]: C++11 introduces the lambda [](){ } allows to write an inline, anonymous function.
    [] captures a value [x] or a reference [&x] or any variable currently
      in scope by reference [&] or by value [=]
--- framed = (msg.initEvent().*(init_model_func))();
      msg.initEvent() // MessageBuilder.initEvent() in messaging.hpp
      cereal::Event::Builder initEvent() // initEvent(): object of Event::Builder in log.capnp.h
      ::cereal::ModelData::Builder initModel(); // initModel(): object of ModelData
      framed == (msg.initEvent().*(init_model_func))() is an object of ModelData
      framed.setRawPred() in log.capnp.h

/OP081/selfdrive/modeld/models/commonmodel.h
  const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;
  void softmax(const float* input, float* output, size_t len);
  typedef struct ModelFrame {
    Transform transform;
    int transformed_width, transformed_height;
    cl_mem transformed_y_cl, transformed_u_cl, transformed_v_cl;
    LoadYUVState loadyuv;
    cl_mem net_input;
    size_t net_input_size;
  } ModelFrame;
  void frame_init(ModelFrame* frame, int width, int height,

--- getenv: see CAN++ 4

/OP081/selfdrive/modeld/models/commonmodel.cc
  #include "commonmodel.h"
----- Input 2: frame_init.transform_init().loadyuv_init(loadyuv,,,width,height)
  void frame_init(ModelFrame* frame, int width, int height,
                        cl_device_id device_id, cl_context context) {
    transform_init(&frame->transform, context, device_id);
    frame->transformed_width = width;
    frame->transformed_height = height;
    loadyuv_init(&frame->loadyuv, context, device_id, frame->transformed_width, frame->transformed_height);

--- transform_init() defined in /selfdrive/modeld/transforms/transform.cc
    loadyuv_init() defined in /selfdrive/modeld/transforms/loadyuv.cc

/OP081/selfdrive/modeld/runners/run.h
  #include "runmodel.h"
  #include "snpemodel.h"
  #ifdef QCOM
----- Load Model: model_init.DefaultRunModel=SNPEModel(path/supercombo, &out, size, runtime)
    #define DefaultRunModel SNPEModel

/OP081/selfdrive/modeld/runners/runmodel.h
----- Execute Model: model_eval_frame.s->m->RunModel.execute(*, )
  class RunModel {
    virtual void addRecurrent(float *state, int state_size) {}
    virtual void addDesire(float *state, int state_size) {}
    virtual void addTrafficConvention(float *state, int state_size) {}
    virtual void execute(float *net_input_buf, int buf_size) {}};

/OP081/selfdrive/modeld/runners/snpemodel.h
  #include <SNPE/SNPE.hpp>
  #include <SNPE/SNPEBuilder.hpp>
  #include "runmodel.h"
  #define USE_CPU_RUNTIME 0
  #define USE_GPU_RUNTIME 1
  #define USE_DSP_RUNTIME 2
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.execute(*, )
  class SNPEModel : public RunModel {
----- Load Model: model_init.DefaultRunModel=SNPEModel(path/supercombo, &out, size, runtime)
    SNPEModel(const char *path, float *loutput, size_t loutput_size, int runtime);
    void addRecurrent(float *state, int state_size);
    void addTrafficConvention(float *state, int state_size);
    void addDesire(float *state, int state_size);
    void execute(float *net_input_buf, int buf_size);
    zdl::DlSystem::Runtime_t Runtime;
    std::unique_ptr<zdl::SNPE::SNPE> snpe;
    zdl::DlSystem::UserBufferMap inputMap;
    std::unique_ptr<zdl::DlSystem::IUserBuffer> inputBuffer;
    zdl::DlSystem::UserBufferMap outputMap;
    std::unique_ptr<zdl::DlSystem::IUserBuffer> outputBuffer;
    float *output;

/OP081/selfdrive/modeld/runners/snpemodel.cc
  #include "snpemodel.h"
  SNPEModel::SNPEModel(const char *path, float *loutput, size_t loutput_size, int runtime) {
    if (runtime==USE_GPU_RUNTIME) {
      Runtime = zdl::DlSystem::Runtime_t::GPU;
      Runtime = zdl::DlSystem::Runtime_t::DSP;
      Runtime = zdl::DlSystem::Runtime_t::CPU;}
    size_t model_size;
----- Load Model: model_init.DefaultRunModel=SNPEModel(path/supercombo.dlc,,,).read_file(path)
    model_data = (uint8_t *)read_file(path, &model_size);
    // load model
    std::unique_ptr<zdl::DlContainer::IDlContainer> container = zdl::DlContainer::IDlContainer::open(model_data, model_size);
    printf("loaded model with size: %lu\n", model_size);
    zdl::SNPE::SNPEBuilder snpeBuilder(container.get());
    while (!snpe) {
      snpe = snpeBuilder.setOutputLayers({}) // create model runner
                        .setRuntimeProcessor(Runtime)
                        .build();
    const auto &strListi_opt = snpe->getInputTensorNames(); // get input and output names
    const char *input_tensor_name = strListi.at(0);
    const auto &strListo_opt = snpe->getOutputTensorNames();
    const char *output_tensor_name = strListo.at(0);
    printf("model: %s -> %s\n", input_tensor_name, output_tensor_name);
    // create input buffer
      const auto &inputDims_opt = snpe->getInputDimensions(input_tensor_name);
      const zdl::DlSystem::TensorShape& bufferShape = *inputDims_opt;
      for (size_t i = 0; i < bufferShape.rank(); i++) product *= bufferShape[i];
      printf("input product is %lu\n", product);
      inputBuffer = ubFactory.createUserBuffer(NULL, product*sizeof(float), strides, &userBufferEncodingFloat);
    // create output buffer
      const zdl::DlSystem::TensorShape& bufferShape = snpe->getInputOutputBufferAttributes(output_tensor_name)->getDims();
      outputBuffer = ubFactory.createUserBuffer(output, output_size * sizeof(float), outputStrides, &userBufferEncodingFloat);
  void SNPEModel::addRecurrent(float *state, int state_size) {
  void SNPEModel::addTrafficConvention(float *state, int state_size) {
  void SNPEModel::addDesire(float *state, int state_size) {
  std::unique_ptr<zdl::DlSystem::IUserBuffer> SNPEModel::addExtra(float *state, int state_size, int idx) {
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.snpe->execute(Map, Map)
  void SNPEModel::execute(float *net_input_buf, int buf_size) {
      float *inputs[4] = {recurrent, trafficConvention, desire, net_input_buf};
      if (!snpe->execute(inputMap, outputMap)) {
        PrintErrorStringAndExit();}

--- !snpe: !ptr evaluates true or false (bool) depending on ptr is nullptr or not.
    https://stackoverflow.com/questions/51398223/pointer-negation-in-c-ptr-null/51399044#51399044
--- at(0): array::at() is a built-in function in C++ STL which returns a reference
      to the element present at location i in given array.
      array<int, 5> arr = { 1, 5, 2, 4, 7 }; => arr.at(1) == 5
    https://www.geeksforgeeks.org/array-at-function-in-c-stl/

/OP081/phonelibs/snpe/include/SNPE/SNPE.hpp
  #include "DlSystem/ITensor.hpp"
  #include "DlSystem/TensorShape.hpp"
  #include "DlSystem/TensorMap.hpp"
  #include "DlSystem/ZdlExportDefine.hpp"
  namespace zdl { namespace SNPE {
  class ZDL_EXPORT SNPE final // final class cannot be inherited
  {
     explicit SNPE(std::unique_ptr<zdl::SNPE::SnpeRuntime>&& runtime) noexcept;
     zdl::DlSystem::Optional<zdl::DlSystem::StringList>
        getInputTensorNames() const noexcept;
     zdl::DlSystem::Optional<zdl::DlSystem::StringList>
        getOutputTensorNames() const noexcept;
     zdl::DlSystem::StringList
        getOutputTensorNamesByLayerName(const char *name) const noexcept;
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.SNPE.execute(TensorMap &, TensorMap &)
----- Execute Model: final execute() defined in some SNPE.obj by ZDL_EXPORT??? (SNPE.c is not found)
     bool execute(const zdl::DlSystem::TensorMap &input,
                  zdl::DlSystem::TensorMap &output) noexcept;
     zdl::DlSystem::Optional<zdl::DlSystem::TensorShape>
        getInputDimensions() const noexcept;
     zdl::DlSystem::Optional<zdl::DlSystem::TensorShape>
        getInputDimensions(const char *name) const noexcept;

--- zdl::SNPE::SNPE: zdl is a namespace, SNPE is a namespace, SNPE is a class.
    https://www.geeksforgeeks.org/difference-namespace-class/
--- const noexcept: A C++ exception is a response to an exceptional circumstance
     that arises while a program is running, such as an attempt to divide by zero.
    noexcept is a guarantee that your function will not throw any exceptions.
      If it does, the program is terminated.
    f() const does not modify the object on which f() is called.
    https://www.tutorialspoint.com/cplusplus/cpp_exceptions_handling.htm
    https://stackoverflow.com/questions/60615588/how-to-convert-a-virtual-function-with-const-noexcept-from-c11-to-c17
    https://www.geeksforgeeks.org/const-member-functions-c/
--- &&: In C++11. int&& a means "a" is an r-value reference.
    https://stackoverflow.com/questions/4549151/c-double-address-operator/4549169
    http://thbecker.net/articles/rvalue_references/section_01.html

/OP081/selfdrive/common/visionbuf_ion.c
  #include <linux/ion.h>
  #include <CL/cl_ext.h>
  #include "visionbuf.h"
VisionBuf visionbuf_allocate(size_t len) {
  return (VisionBuf){
    .len = len,
    .addr = addr,
    .fd = ion_fd_data.fd,
----- Input 2: yuv_ion.visionbuf_allocate_cl.visionbuf_allocate
  VisionBuf visionbuf_allocate_cl(size_t len, cl_device_id device_id, cl_context ctx) {
    VisionBuf buf = visionbuf_allocate(len);
    buf.buf_cl = CL_CHECK_ERR(clCreateBuffer(ctx,
    return buf;
  void visionbuf_free(const VisionBuf* buf) {

--- yuv_ion: ion.h: ION is the memory manager of Android, it could be used by graphic
      and multimedia stacks to allocate buffers.
    https://www.programmersought.com/article/42787435633/
    https://blog.csdn.net/lunhui2016/article/details/111797978

/OP081/selfdrive/common/visionipc.h
  typedef enum VisionStreamType {
    VISION_STREAM_RGB_FRONT,
    VISION_STREAM_YUV_FRONT,
    VISION_STREAM_MAX,
  } VisionStreamType;
  typedef struct VisionStreamBufs {
    VisionStreamType type;
    int width, height, stride;
    size_t buf_len;
    union {
      VisionUIInfo ui_info;
    } buf_info;
  } VisionStreamBufs;
  typedef struct VIPCBufExtra { // only for yuv
    uint32_t frame_id;
    uint64_t timestamp_sof;
    uint64_t timestamp_eof;
  } VIPCBufExtra;
  typedef struct VIPCBuf {
    int fd;
    size_t len;
    void* addr;
  } VIPCBuf;
  typedef struct VisionStream {
    int ipc_fd;
    int last_idx;
    int last_type;
    int num_bufs;
    VisionStreamBufs bufs_info;
    VIPCBuf *bufs;
  } VisionStream;
  int visionstream_init(VisionStream *s, VisionStreamType type, bool tbuffer, VisionStreamBufs *out_bufs_info);
  VIPCBuf* visionstream_get(VisionStream *s, VIPCBufExtra *out_extra);

--- IPC: Inter-process communication allows processes to communicate with
      each other and synchronize their actions. Transfer native objects between two processes
    https://www.tutorialspoint.com/what-is-interprocess-communication

/OP081/selfdrive/common/visionbuf.h
  #include <OpenCL/cl.h>
  typedef struct VisionBuf {
    cl_context ctx;
    cl_device_id device_id;
    cl_mem buf_cl;
  } VisionBuf;
  VisionBuf visionbuf_allocate_cl(size_t len, cl_device_id device_id, cl_context ctx);

--- OpenCL (Open Computing Language) is a framework for writing programs that
      execute across heterogeneous platforms CPUs, GPUs, DSPs, etc.
    https://www.nersc.gov/assets/pubs_presos/MattsonTutorialSC14.pdf

/OP081/selfdrive/common/modeldata.h
  constexpr int MODEL_PATH_DISTANCE = 192;
  constexpr int  TRAJECTORY_SIZE = 33;
  constexpr float MIN_DRAW_DISTANCE = 10.0;
  constexpr float MAX_DRAW_DISTANCE = 100.0;
  constexpr int POLYFIT_DEGREE = 4;
  constexpr int SPEED_PERCENTILES = 10;
  constexpr int DESIRE_PRED_SIZE = 32;
  constexpr int OTHER_META_SIZE = 4;

/OP081/cereal/messaging/socketmaster.cc
  #include "messaging.hpp"
  #include "services.h"
  static const service *get_service(const char *name) {
    for (const auto &it : services) {
      if (strcmp(it.name, name) == 0) return &it;
  struct SubMaster::SubMessage {
    kj::Array<capnp::word> buf;
  SubMaster::SubMaster(const std::initializer_list<const char *> &service_list, const char *address,
  int SubMaster::update(int timeout) {
      memcpy(m->buf.begin(), msg->getData(), msg->getSize());
  PubMaster::PubMaster(const std::initializer_list<const char *> &service_list) {
    for (auto name : service_list) {
      PubSocket *socket = PubSocket::create(ctx.ctx_, name);
----- Output: ModelData: pm.send(*name, &msg).send(1, 2, 3)
  int PubMaster::send(const char *name, MessageBuilder &msg) {
    auto bytes = msg.toBytes();
    return send(name, bytes.begin(), bytes.size());

--- static const service *get_service: static means the value of get_service is
      shared between all instances of the class service and const means it doesn't change.
      get_service returns the struct service type.
--- for (auto &it1 : a) cin >> it1; == for(int i = 0; i < n; i++){cin>>a[i];}
      Here a is an array or vector of any type. it1 has automatically the same type of a.
      &it1 means that it1 takes the references (addresses) of a[i] and works as it1 instead of a[i].
    https://stackoverflow.com/questions/63279038/what-does-forauto-it-a-mean
--- strcmp(it.name, name): https://www.cplusplus.com/reference/cstring/strcmp/
--- std::memcpy() copies a block of memory from a location to another.
      memcpy (str1, str2, sizeof(str2)); // Copies contents of str2 to str1

/OP081/cereal/messaging/messaging.cc
  #include "messaging.hpp"
  #include "impl_zmq.h"
  SubSocket * SubSocket::create(){
    SubSocket * s;
      s = new ZMQSubSocket();
    return s;}
  PubSocket * PubSocket::create(){
    PubSocket * s;
    if (std::getenv("ZMQ") || MUST_USE_ZMQ){
      s = new ZMQPubSocket();
    return s;

/OP081/cereal/messaging/impl_zmq.hpp
  #include "messaging.hpp"
  #include <zmq.h>
  class ZMQContext : public Context {
  class ZMQMessage : public Message {
    size_t getSize(){return size;}
    char * getData(){return data;}
  class ZMQSubSocket : public SubSocket {
    int connect(Context *context, std::string endpoint, std::string address, bool conflate=false);
    void setTimeout(int timeout);
    void * getRawSocket() {return sock;}
    Message *receive(bool non_blocking=false);
  class ZMQPubSocket : public PubSocket {
    void * sock;
    std::string full_endpoint;
    int connect(Context *context, std::string endpoint);
    int sendMessage(Message *message);
----- Output: ModelData: pm.send(1, 2).send(1, 2, 3).PubSocket.ZMQPubSocket.send(1, 2)
    int send(char *data, size_t size);
  class ZMQPoller : public Poller {
    void registerSocket(SubSocket *socket);
--- The poller notifies when there's data (messages) available on the sockets;
      it's your job to read it.
    If none of the requested events have occurred on any zmq_pollitem_t item,
      zmq_poll() shall wait timeout microseconds for an event to occur on any
      of the requested items.
    poller.poll(1000) blocks for 1s, then times out.
    If the value of timeout is 0, zmq_poll() shall return immediately.
    zmq_poll() function returns the number of zmq_pollitem_t structures
      with events signaled in revents, 0 if no events have been signaled,
      and -1 if failure.
    zmq_poll(): http://api.zeromq.org/2-1:zmq-poll
    https://stackoverflow.com/questions/18114343/how-does-zmq-poller-work

/OP081/cereal/messaging/impl_zmq.cc
  #include <zmq.h>
  #include "services.h"
  #include "impl_zmq.hpp"
  static int get_port(std::string endpoint) {
    for (const auto& it : services) {
      std::string name = it.name;
      if (name == endpoint) { port = it.port; break;
    return port;
  ZMQContext::ZMQContext() { context = zmq_ctx_new();
  int ZMQSubSocket::connect(Context *context, std::string endpoint, std::string address, bool conflate, bool check_endpoint){
    sock = zmq_socket(context->getRawContext(), ZMQ_SUB);
    full_endpoint = "tcp://" + address + ":";
    full_endpoint += std::to_string(get_port(endpoint));
    return zmq_connect(sock, full_endpoint.c_str());
  Message * ZMQSubSocket::receive(bool non_blocking){
    zmq_msg_t msg;
    int rc = zmq_msg_recv(&msg, sock, flags);
    Message *r = NULL;
    if (rc >= 0){
      // Make a copy to ensure the data is aligned
      r = new ZMQMessage;
      r->init((char*)zmq_msg_data(&msg), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return r;
  int ZMQPubSocket::sendMessage(Message *message){
    return zmq_send(sock, message->getData(), message->getSize(), ZMQ_DONTWAIT);
----- Output: ModelData: pm.send(1, 2).send(1, 2, 3).PubSocket.ZMQPubSocket.send(1, 2).zmq_send(1, 2, 3, 4)
  int ZMQPubSocket::send(char *data, size_t size){
    return zmq_send(sock, data, size, ZMQ_DONTWAIT);

--- How To Work with the ZeroMQ Messaging Library
    ZMQ: ZeroMQ (ØMQ, 0MQ, 0 message queue) is an asynchronous messaging system
      toolkit (i.e. a library) between applications and processes - fast and asynchronously.
    tcp (Transmission Control Protocol) impl (implementation)
    In computer networking, a port is a communication endpoint.
    A socket is externally identified to other hosts by its socket address, which is
      the triad of transport protocol, type, IP address, and port number. Example:
      Socket mysocket = getSocket(type = "TCP")
      connect(mysocket, address = "1.2.3.4", port = "80")
      send(mysocket, "Hello, world!")
      close(mysocket)
    Define the socket using the "Context"
--- zmq_connect(void *socket, const char *endpoint) is an zmq function (zmq.h) that connects
      the socket to an endpoint and then accepts incoming connections on that endpoint.
    http://api.zeromq.org/3-2:zmq-connect
    http://api.zeromq.org/
--- zmq_send(void *socket, void *buf, size_t len, int flags)  shall queue a message
      created from the buffer referenced by the buf and len arguments.
    http://api.zeromq.org/master:zmq-send

/OP081/cereal/gen/cpp/log.capnp.h
  // Generated by Cap'n Proto compiler, DO NOT EDIT, source: log.capnp
  #include "car.capnp.h"
  namespace capnp {
  namespace schemas {
  }  // namespace schemas
  }  // namespace capnp
  namespace cereal { // Line 515 in log.capnp.h
    struct ModelData { // Line 1022
      ModelData() = delete;
      class Reader;
      class Builder;
      class Pipeline;
      struct PathData;
      struct LeadData;
      struct ModelSettings;
      struct MetaData;
      struct LongitudinalData;
      struct _capnpPrivate {
        CAPNP_DECLARE_STRUCT_HEADER(b8aad62cffef28a9, 4, 11)
        #if !CAPNP_LITE
        static constexpr ::capnp::_::RawBrandedSchema const* brand() { return &schema->defaultBrand; }
        #endif  // !CAPNP_LITE
      };};
    struct ModelData::PathData { // Line 1042
      PathData() = delete;
      class Reader;
      class Builder;
      class Pipeline;
      struct _capnpPrivate {
        CAPNP_DECLARE_STRUCT_HEADER(8817eeea389e9f08, 2, 3)
    struct ModelData::LeadData { // Line 1057
      LeadData() = delete;
      class Reader;
    class FrameData::Reader { // Line 3602
      Reader() = default;
      inline explicit Reader(::capnp::_::StructReader base): _reader(base) {}
----- Input 1: sm["frame"].capnp.getFrame().getFrameId()
      inline  ::uint32_t getFrameId() const; // Line 3632
      inline  ::capnp::Data::Reader getImage() const;
    class ModelData::Builder { // Line 6742
      typedef ModelData Builds;
      Builder() = delete;  // Deleted to discourage incorrect usage.
                           // You can explicitly initialize to nullptr instead.
      inline Builder(decltype(nullptr)) {}
      inline explicit Builder(::capnp::_::StructBuilder base): _builder(base) {}
      inline operator Reader() const { return Reader(_builder.asReader()); }
      inline Reader asReader() const { return *this; }
      inline ::capnp::MessageSize totalSize() const { return asReader().totalSize(); }
    #if !CAPNP_LITE
      inline ::kj::StringTree toString() const { return asReader().toString(); }
    #endif  // !CAPNP_LITE
      inline  ::uint32_t getFrameId(); // Line 6758
      inline void setFrameId( ::uint32_t value);
      inline void setPath( ::cereal::ModelData::PathData::Reader value);
      inline  ::cereal::ModelData::PathData::Builder initPath();
      inline  ::cereal::ModelData::PathData::Builder initLeftLane();
      inline void setRawPred( ::capnp::Data::Reader value);
    }; // class ModelData::Builder, Line 6862
    inline  ::cereal::PathPlan::Desire getDesire() const; // Line 8876
    inline  ::cereal::PathPlan::Reader getPathPlan() const; // Line 17835
    class Event::Reader { // Line 17561
----- Input 1: sm["frame"].capnp.getFrame().getFrameId()
      inline  ::cereal::FrameData::Reader getFrame() const; // Line 17587
    }; // class Event::Builder, Line 18537
    class Event::Builder { // Line 17899
      inline  ::cereal::FrameData::Builder getFrame(); // Line 17929
      inline  ::cereal::ModelData::Builder initModel(); // Line 17987
      inline  ::cereal::ModelDataV2::Builder initModelV2(); // Line 18510
    }; // class Event::Builder, Line 18537
  }  // namespace cereal, Line 45862

/OP081/cereal/log.capnp
  using Cxx = import "./include/c++.capnp";
  $Cxx.namespace("cereal");
  struct FrameData {
    frameId @0 :UInt32;
    timestampEof @2 :UInt64;
    frameLength @3 :Int32;
    integLines @4 :Int32;
    globalGain @5 :Int32;
    lensPos @11 :Int32;
    lensSag @12 :Float32;
    lensErr @13 :Float32;
    lensTruePos @14 :Float32;
    image @6 :Data;
    gainFrac @15 :Float32;
    focusVal @16 :List(Int16);
    focusConf @17 :List(UInt8);
    sharpnessScore @18 :List(UInt16);
    recoverState @19 :Int32;
    frameType @7 :FrameType;
    timestampSof @8 :UInt64;
    transform @10 :List(Float32);
    androidCaptureResult @9 :AndroidCaptureResult;
    enum FrameType {
      unknown @0;
      neo @1;
      chffrAndroid @2;
      front @3;}
    struct AndroidCaptureResult {
      sensitivity @0 :Int32;
      frameDuration @1 :Int64;
      exposureTime @2 :Int64;
      rollingShutterSkew @3 :UInt64;
      colorCorrectionTransform @4 :List(Int32);
      colorCorrectionGains @5 :List(Float32);
      displayRotation @6 :Int8;}
  }
  struct CanData {
    address @0 :UInt32;
    busTime @1 :UInt16;
    dat     @2 :Data;
    src     @3 :UInt8;
  struct ModelData {
    frameId @0 :UInt32;
    rawPred @15 :Data;
    path @1 :PathData;
    leftLane @2 :PathData;
    rightLane @3 :PathData;
    lead @4 :LeadData;
    freePath @6 :List(Float32);
    leadFuture @7 :LeadData;
    speed @8 :List(Float32);
    meta @10 :MetaData;
    longitudinal @11 :LongitudinalData;
    struct PathData {
      points @0 :List(Float32);
      prob @1 :Float32;
      std @2 :Float32;
      stds @3 :List(Float32);
      poly @4 :List(Float32);
    struct LeadData {
      dist @0 :Float32;
      prob @1 :Float32;
      std @2 :Float32;
      relVel @3 :Float32;
      relVelStd @4 :Float32;
      relY @5 :Float32;
      relYStd @6 :Float32;
      relA @7 :Float32;
      relAStd @8 :Float32;
    struct ModelSettings {
    struct MetaData {
    struct LongitudinalData {
      distances @2 :List(Float32);
      speeds @0 :List(Float32);
      accelerations @1 :List(Float32);
  struct Event {
    logMonoTime @0 :UInt64;  # in nanoseconds?
    union {
      frame @2 :FrameData;
      can @5 :List(CanData);
      model @9 :ModelData;
      sendcan @17 :List(CanData);

--- Float32, Data, Text, UInt32, List(), Type, ...: Capnp Built-in Types
    https://capnproto.org/language.html#built-in-types
--- struct Event {} contains the most important data

/OP081/cereal/services.h
  struct service { char name[0x100]; int port; bool should_log; int frequency; int decimation; };
  static struct service services[] = {
    { .name = "frame", .port = 8002, .should_log = true, .frequency = 20, .decimation = 1 },
    { .name = "can", .port = 8006, .should_log = true, .frequency = 100, .decimation = -1 },
    { .name = "model", .port = 8009, .should_log = true, .frequency = 20, .decimation = 5 },
    { .name = "sendcan", .port = 8017, .should_log = true, .frequency = 100, .decimation = -1 },
    { .name = "plan", .port = 8024, .should_log = true, .frequency = 20, .decimation = 2 },
    { .name = "pathPlan", .port = 8067, .should_log = true, .frequency = 20, .decimation = 2 },
    { .name = "modelV2", .port = 8077, .should_log = true, .frequency = 20, .decimation = 20 },

--- services[] contains a list of Event names
