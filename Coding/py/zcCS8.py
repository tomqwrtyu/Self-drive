CAN 20   JLL, 2021.7.23-8.2, 9.10

Read YP's Code in OP079C2
Read zcCS3a.py first
----- Keywords: steerd, SteerData, myModelState, mymodel, steermodel, initSteermodel, Mat
--------------------------------------------------

---------- Part B: Read YP's Code in OP079C2

--- read code   /OP079C2 == /data/openpilot
--- # or // means the original code (/OP079) line being changed or commented out or deleted

--- 3. rebuild OP: Read Code by Atom Editor Split Diff between
/OP079/SConstruct
/OP079C2/SConstruct
  SConscript(['selfdrive/steerd/SConscript'])

--- SConstruct & SConscript:
    https://drive.google.com/file/d/1VznxlVXSCHjoE2KAUZfuYJfpnoNQQjuG/view

--- Split Diff
/OP079C2/selfdrive/steerd/SConscript
/OP079/selfdrive/modeld/SConscript
  libs = ['opencv_gapi'???,'opencv_imgproc','opencv_stitching','opencv_core','opencv_highgui','opencv_aruco','opencv_bgsegm','opencv_bioinspired','opencv_ccalib',
          cereal, messaging, common, 'OpenCL'???, 'SNPE', 'capnp', 'zmq', 'kj', 'yuv', gpucommon, visionipc]
  #libs = [cereal, messaging, common, 'OpenCL', 'SNPE', 'symphony-cpu'???, 'capnp', 'zmq', 'kj', 'yuv', gpucommon, visionipc]
  common_src = [  "steers/commonmodel.c",
      lenv['CFLAGS']???
      #lenv['CFLAGS'].append("-DUSE_THNEED")
    libs += ['gsl', 'CB', 'symphony-cpu', 'pthread']???
    #libs += ['gsl', 'CB', 'pthread']
    libs += ['symphony-cpu', 'pthread']???
    #libs += ['pthread']
  common = lenv.Object(common_src)
  lenv.ParseConfig("pkg-config opencv4 --cflags")???
  lenv.Program('testopencv',["testopencv.cc"],LIBS=libs);
  lenv.Program('_steermodeld', [
    "steermodeld.cc",  "steers/steerd.cc",  ]+common, LIBS=libs)

--- Qualcomm® Symphony runtime handles the overall system scheduling and power management
      of parallel processing and heterogeneous computing on CPU, GPU, and DSP.
    https://developer.qualcomm.com/blog/heterogeneous-computing-made-simpler-symphony-sdk

--- 6. run uiview.py
/OP079/selfdrive/manager.py
/OP079C2/selfdrive/manager.py
  #"updated": "selfdrive.updated",   # turns off automatic updates of OP versions
  "steerd":("selfdrive/steerd",["./steermodeld"]),

/OP079C2/selfdrive/steerd
--- new folder

/OP079C2/selfdrive/status??? how to get this file

/OP079C2/selfdrive/debug/uiview.py
/OP079/selfdrive/debug/uiview.py
  from selfdrive.manager import start_managed_process, kill_managed_process
  procs = ['camerad', 'ui', 'steerd', 'calibrationd']
  #procs = ['camerad', 'ui', 'modeld', 'calibrationd']
  dat_cs, dat_thermal = [messaging.new_message(s) for s in services]
  #dat_cs, dat_thermal, dat_radar = [messaging.new_message(s) for s in services]
    #pm.send('radarState', dat_radar)

/OP079C2/selfdrive/steerd/steermodeld
/OP079/selfdrive/modeld/modeld
  export LD_LIBRARY_PATH="/data/pythonpath/phonelibs/snpe/aarch64/:$LD_LIBRARY_PATH"
  exec ./_steermodeld
/OP079C2/selfdrive/steerd/_steermodeld

/OP079C2/selfdrive/steerd/steermodeld.cc
/OP079/selfdrive/modeld/modeld.cc
  #include "steers/steerd.h"
  #include "messaging.hpp"
  void* live_thread(void *arg) {
    SubMaster sm({"liveCalibration"});
    while (!do_exit) {
      if (sm.update(10) > 0){
        run_model = true;
  int main(int argc, char **argv) {
    pthread_mutex_init(&transform_lock, NULL);
    // start calibration thread
    err = pthread_create(&live_thread_handle, NULL, live_thread, NULL);
    // messaging
----- Input 1: sm({"frame"})
----- Output: pm({"mymodel"})
    PubMaster pm({"mymodel","radarState"});
    SubMaster sm({"frame"});
----- Load Model: mymodel_init(&model)
    myModelState model;
    mymodel_init(&model, device_id, context);
    VisionStream stream;
    while (!do_exit) { // while 1
      VisionStreamBufs buf_info;
      err = visionstream_init(&stream, VISION_STREAM_YUV, true, &buf_info);
      //VisionBuf yuv_ion = visionbuf_allocate_cl(buf_info.buf_len, device_id, context, &yuv_cl);
      uint32_t frame_id = 0, last_vipc_frame_id = 0;
      while (!do_exit) { // while 2
        VIPCBuf *buf;
        VIPCBufExtra extra;
        buf = visionstream_get(&stream, &extra);
        //mat3 model_transform = cur_transform;
        const bool run_model_this_iter = run_model;
        if (sm.update(0) > 0){
----- Input 1: sm["frame"].getFrame().getFrameId()
          frame_id = sm["frame"].getFrame().getFrameId();}
        if (run_model_this_iter) {
          //memcpy(yuv_ion.addr, buf->addr, buf_info.buf_len);
----- Execute Model: SteerData=model_eval_frame(&model)
        SteerData model_buf =  model_eval_frame(&model, q, buf);
          model_publish(pm, extra.frame_id, frame_id,  vipc_dropped_frames, frame_drop_perc, model_buf, extra.timestamp_eof);
          //posenet_publish(pm, extra.frame_id, frame_id, vipc_dropped_frames, frame_drop_perc, model_buf, extra.timestamp_eof);
          LOGD("model process: %.2fms, from last %.2fms, vipc_frame_id %zu, frame_id, %zu, frame_drop %.3f", mt2-mt1, mt1-last, extra.frame_id, frame_id, frame_drop_ratio);}
      } // while 2 end
      //visionbuf_free(&yuv_ion);
      visionstream_destroy(&stream);}
    } // while 1 end
    model_free(&model);}

/OP079C2/selfdrive/steerd/steers/steerd.cc??? why d
/OP079/selfdrive/modeld/models/driving.cc
  #include <math.h>
  #include "steerd.h"
  Mat getFlatVector(VIPCBuf  *buf) {
      uint8_t *y = (uint8_t*)buf->addr;
      Mat src, rgbsrc, dst, rgb_s;
      src = Mat(874*3/2, 1164, CV_8UC1, y);
      rgbsrc = Mat(874, 1164, CV_8UC3);
      cvtColor(src, rgbsrc, COLOR_YUV2RGB_IYUV);
      rgb_s = rgbsrc(Rect(0, 0, 1164, 650));
      Size dsize = Size(320, 160);
----- Input 2: frame_init(&s->frame,,,,context)
----- Load Model: mymodel_init.s->m=DefaultRunModel(path/ypreg.dlc, &out,,)
  void mymodel_init(myModelState* s, cl_device_id device_id, cl_context context){
    frame_init(&s->frame, MODEL_WIDTH, MODEL_HEIGHT, device_id,context);
    s->input_frame = (float*)calloc(MODEL_FRAME_SIZE*2, sizeof(float));
    const int output_size = 105;
    s->output = (float*)calloc(output_size, sizeof(float));
    s->m = new DefaultRunModel("../../models/ypreg.dlc", s->output, output_size, USE_GPU_RUNTIME);
----- Execute Model: model_eval_frame.s->m->execute(&, )
  SteerData model_eval_frame(myModelState * s, cl_command_queue q, VIPCBuf *buf){
     Mat rgb_frame_buf = getFlatVector(buf);
     float *data= (float*)malloc(320*160*3*sizeof(float));
     for (int i=0;i<320*160*3;i++){
       data[i]=float(rgb_frame_buf.data[i]);}
     memmove(&s->input_frame[0], data, sizeof(float)*MODEL_FRAME_SIZE*2);
     s->m->execute(s->input_frame, MODEL_FRAME_SIZE*2);
     SteerData net_outputs;
     rgb_frame_buf.release();
     delete data;
     net_outputs.point = &s->output[0];
     net_outputs.valid_len= &s->output[50];
     net_outputs.error= &s->output[51];
     net_outputs.radar= &s->output[101];
     return net_outputs;
----- Output: SteerData: pm.send(pub_name=="radarState", msgg==SteerData) defined in socketmaster.cc
----- Output: SteerData: pm.send(pub_name=="mymodel", msg==SteerData) defined in socketmaster.cc
  void mymodel_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t frame_id,
                   uint32_t vipc_dropped_frames, float frame_drop, const SteerData &net_outputs, uint64_t timestamp_eof) {
    MessageBuilder msg;
    MessageBuilder msgg;
    poly_fit(points, stds, poly_arr, point_arr, valid_len);
    auto steerd = msg.initEvent().initSteermodel();
    auto leadone = msgg.initEvent().initRadarState().initLeadOne();
    pm.send("radarState", msgg);
    pm.send("mymodel", msg);

--- steermodel declared in struct Event{} in log.capnp.
    <capnp/serialize.h> generates initSteermodel() in log.capnp.h by changing
      "st..." to "St..." (see also radarState, leadOne in log.capnp).

/OP079C2/selfdrive/steerd/steers/steerd.h
/OP079/selfdrive/modeld/models/driving.h
  #include "runners/run.h"
  #include <opencv2/opencv.hpp>
  struct SteerData {
         float* point;
         float* valid_len;
         float* error;
         float* radar;};
  typedef struct myModelState {
    ModelFrame frame;
    float* output;
    float* input_frame;
----- Load Model: mymodel_init.s->m=DefaultRunModel=RunModel(path/ypreg.dlc, &out,,)
    RunModel *m;} myModelState;
  void mymodel_init(myModelState* s,cl_device_id device_id,cl_context context );
  SteerData model_eval_frame(myModelState *s,cl_command_queue q,VIPCBuf *buf);
  cv::Mat getFlatVector(VIPCBuf *buf);
  void mymodel_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t frame_id,
                   uint32_t vipc_dropped_frames, float frame_drop, const SteerData &net_outputs, uint64_t timestamp_eof);

--- SteerData also in log.capnp.
--- cv::Mat: The class Mat represents an n-dimensional dense numerical single-channel or multi-channel array.
    Mat (int rows, int cols, int type, const Scalar &s)
    Mat (int rows, int cols, int type)
    https://docs.opencv.org/4.5.2/d3/d63/classcv_1_1Mat.html#details

/OP079C2/selfdrive/steerd/runners/run.h
  #include "runmodel.h"
  #include "snpemodel.h"
  #ifdef QCOM
----- Load Model: mymodel_init.s->m=DefaultRunModel=RunModel=SNPEModel(ypreg.dlc, output)
    #define DefaultRunModel SNPEModel

/OP079C2/selfdrive/steerd/runners/runmodel.h
----- Load Model: mymodel_init.s->m=DefaultRunModel=RunModel=SNPEModel(ypreg.dlc, output)
  class RunModel {
  public:
    virtual void addRecurrent(float *state, int state_size) {}
    virtual void addDesire(float *state, int state_size) {}
    virtual void addTrafficConvention(float *state, int state_size) {}
    virtual void execute(float *net_input_buf, int buf_size) {}};

/OP079C2/selfdrive/steerd/runners/snpemodel.h
  #include <SNPE/SNPE.hpp>
  #include <SNPE/SNPEBuilder.hpp>
  #include "runmodel.h"
----- Load Model: mymodel_init.s->m=DefaultRunModel=RunModel=SNPEModel(ypreg.dlc, output)
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.execute(*, )
  class SNPEModel : public RunModel {
    SNPEModel(const char *path, float *loutput, size_t loutput_size, int runtime);
    void execute(float *net_input_buf, int buf_size);
    std::unique_ptr<zdl::SNPE::SNPE> snpe;

/OP079C2/selfdrive/steerd/runners/snpemodel.cc
  #include "snpemodel.h"
  SNPEModel::SNPEModel(const char *path, float *loutput, size_t loutput_size, int runtime) {
    if (runtime==USE_GPU_RUNTIME) {
      Runtime = zdl::DlSystem::Runtime_t::GPU;
      Runtime = zdl::DlSystem::Runtime_t::DSP;
      Runtime = zdl::DlSystem::Runtime_t::CPU;}
    size_t model_size;
----- Load Model: mymodel_init.s->m=DefaultRunModel=RunModel=SNPEModel().read_file(path).open(ypreg.dlc)
    model_data = (uint8_t *)read_file(path, &model_size);
    // load model
    std::unique_ptr<zdl::DlContainer::IDlContainer> container = zdl::DlContainer::IDlContainer::open(model_data, model_size);
    printf("loaded model with size: %lu\n", model_size);
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.execute(*, ).snpe->execute(Map, Map)
  void SNPEModel::execute(float *net_input_buf, int buf_size) {
      float *inputs[4] = {recurrent, trafficConvention, desire, net_input_buf};
      if (!snpe->execute(inputMap, outputMap)) {
        PrintErrorStringAndExit();}

/OP079C2/phonelibs/snpe/include/SNPE/SNPE.hpp
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
----- Execute Model: model_eval_frame.s->m->RunModel=SNPEModel.execute(*,).SNPE.execute(TensorMap &, TensorMap &)
----- Execute Model: final execute() defined in some SNPE.obj by ZDL_EXPORT??? (SNPE.c is not found)
     bool execute(const zdl::DlSystem::TensorMap &input, zdl::DlSystem::TensorMap &output) noexcept;

/OP079C2/selfdrive/steerd/steers/commonmodel.h
typedef struct ModelFrame {  // input
  cl_device_id device_id;
  cl_context context;
  Transform transform;
  int transformed_width, transformed_height;
  cl_mem transformed_y_cl, transformed_u_cl, transformed_v_cl;
  LoadYUVState loadyuv;
  cl_mem net_input;
  size_t net_input_size;
} ModelFrame;

void frame_init(ModelFrame* frame, int width, int height,
                      cl_device_id device_id, cl_context context);

/OP079C2/cereal/messaging/messaging.hpp
/OP079/cereal/messaging/messaging.hpp
--- these two files are the same, no change
  #include <capnp/serialize.h>
  #include "../gen/cpp/log.capnp.h"
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
      return event;
    kj::ArrayPtr<capnp::byte> toBytes() {
      heapArray_ = capnp::messageToFlatArray(*this);
      return heapArray_.asBytes();
  class PubMaster {
    PubMaster(const std::initializer_list<const char *> &service_list);
----- Output: ModelData: pm.send(1, 2)
    int send(const char *name, MessageBuilder &msg);
----- Output: ModelData: pm.send(1, 2).send(1, 2, 3).PubSocket.send(1, 2)
    inline int send(const char *name, capnp::byte *data, size_t size)
      { return sockets_.at(name)->send((char *)data, size); }
    std::map<std::string, PubSocket *> sockets_;
  class PubSocket {
    virtual int connect(Context *context, std::string endpoint) = 0;
    virtual int sendMessage(Message *message) = 0;
----- Output: ModelData: pm.send(1, 2).send(1, 2, 3).PubSocket.send(1, 2)
    virtual int send(char *data, size_t size) = 0; // send() defined in impl_zmq.cc
    static PubSocket * create();
    static PubSocket * create(Context * context, std::string endpoint);

/OP079C2/models/ypreg.dlc
--- new ypreg.dlc == mymodel

/OP079C2/cereal/log.capnp
  struct SteerData{
    points @0 :List(Float32);
    validlen @1 :Float32;
    error @2 :List(Float32);
    radarx @3 :Float32;
    radary @4 :Float32;  }
    struct Event {
    union {
      steermodel @76 : SteerData;

/OP079C2/cereal/service_list.yaml
  mymodel : [8078,true, 20., 20]

/OP079C2/cereal/services.h

/OP079C2/selfdrive/ui/ui.cc
/OP079C2/selfdrive/ui/paint.cc
/OP079C2/selfdrive/ui/SConscript

CCCCCCCCCCCCCCCCCCCCCCCC
managed_processes = {
  "thermald": "selfdrive.thermald.thermald",
  "ui": ("selfdrive/ui", ["./ui"]),
}
#proc = managed_processes["thermald"]
#pargs = proc
proc = managed_processes["ui"]
pdir, pargs = proc
print(pdir)
print(pargs)
print(isinstance(proc, str))
