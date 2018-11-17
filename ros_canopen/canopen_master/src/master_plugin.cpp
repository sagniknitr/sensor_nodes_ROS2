#include <class_loader/class_loader.hpp>
#include <socketcan_interface/reader.h>
#include <canopen_master/canopen.h>

#include <set>

namespace canopen {

class ManagingSyncLayer: public SyncLayer {
protected:
    can::CommInterfaceSharedPtr interface_;
    boost::chrono::milliseconds step_, half_step_;

    std::set<void *> nodes_;
    boost::mutex nodes_mutex_;
    std::atomic<size_t> nodes_size_;
    
    virtual void handleShutdown(LayerStatus &status) {
    }

    virtual void handleHalt(LayerStatus &status)  { /* nothing to do */ }
    virtual void handleDiag(LayerReport &report)  { /* TODO */ }
    virtual void handleRecover(LayerStatus &status)  { /* TODO */ }

public:
    ManagingSyncLayer(const SyncProperties &p, can::CommInterfaceSharedPtr interface)
    : SyncLayer(p), interface_(interface), step_(p.period_ms_), half_step_(p.period_ms_/2), nodes_size_(0)
    {
    }

    virtual void addNode(void * const ptr) {
        boost::mutex::scoped_lock lock(nodes_mutex_);
        nodes_.insert(ptr);
        nodes_size_ = nodes_.size();
    }
    virtual void removeNode(void * const ptr)  {
        boost::mutex::scoped_lock lock(nodes_mutex_);
        nodes_.erase(ptr);
        nodes_size_ = nodes_.size();
    }
};

    
class SimpleSyncLayer: public ManagingSyncLayer {
    time_point read_time_, write_time_;
protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            boost::this_thread::sleep_until(read_time_);
            write_time_ += step_;
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            can::Frame frame(properties.header_, 0);
            boost::this_thread::sleep_until(write_time_);
            if(nodes_size_){ //)
                interface_->send(frame);
            }
            read_time_ = get_abs_time(half_step_);
        }
    }

    virtual void handleInit(LayerStatus &status){
        write_time_ = get_abs_time(step_);
        read_time_ = get_abs_time(half_step_);
    }
public:
    SimpleSyncLayer(const SyncProperties &p, can::CommInterfaceSharedPtr interface)
    : ManagingSyncLayer(p, interface) {}
};

class ExternalSyncLayer: public ManagingSyncLayer {
    can::BufferedReader reader_;
protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        can::Frame msg;
        if(current_state > Init){
            if(reader_.readUntil(&msg, get_abs_time(step_))){ // wait for sync
                boost::this_thread::sleep_until(get_abs_time(half_step_)); // shift readout to middle of period
            }
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        // nothing to do here
    }
    virtual void handleInit(LayerStatus &status){
        reader_.listen(interface_, can::MsgHeader(properties.header_));
    }
public:
    ExternalSyncLayer(const SyncProperties &p, can::CommInterfaceSharedPtr interface)
    : ManagingSyncLayer(p, interface), reader_(true,1) {}
};


template<typename SyncType> class WrapMaster: public Master{
    can::CommInterfaceSharedPtr interface_;
public:
    virtual SyncLayerSharedPtr getSync(const SyncProperties &properties){
        return std::make_shared<SyncType>(properties, interface_);
    }
    WrapMaster(can::CommInterfaceSharedPtr interface) : interface_(interface)  {}

    class Allocator : public Master::Allocator{
    public:
        virtual MasterSharedPtr allocate(const std::string &name,  can::CommInterfaceSharedPtr interface){
            return std::make_shared<WrapMaster>(interface);
        }
    };
};

typedef WrapMaster<SimpleSyncLayer> SimpleMaster;
typedef WrapMaster<ExternalSyncLayer> ExternalMaster;
}
CLASS_LOADER_REGISTER_CLASS(canopen::SimpleMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::ExternalMaster::Allocator, canopen::Master::Allocator);

