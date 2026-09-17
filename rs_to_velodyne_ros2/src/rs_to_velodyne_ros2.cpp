#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kNegTimeTolerance = 1e-6;
constexpr uint64_t kUint32Max = static_cast<uint64_t>(UINT32_MAX);

enum class OutType { XYZI, XYZIR, XYZIRT };
enum class ScalarType { None, I8, I16, I32, U8, U16, U32, F32, F64 };
enum class RingSource { Field, OrganizedRows, OrganizedCols };
enum class TimeSource { None, Timestamp, Time };
enum class TimeMode { Absolute, Relative };
enum class TimeUnit { Second, Millisecond, Microsecond, Nanosecond };
enum class TimeOrderPolicy { Validate, Ignore };

inline int scalarSize(ScalarType t) {
    switch (t) {
        case ScalarType::I8:  case ScalarType::U8:  return 1;
        case ScalarType::I16: case ScalarType::U16: return 2;
        case ScalarType::I32: case ScalarType::U32:
        case ScalarType::F32: return 4;
        case ScalarType::F64: return 8;
        default:              return 0;
    }
}

struct FieldDesc {
    int offset = -1;
    ScalarType type = ScalarType::None;
    bool ok() const { return offset >= 0 && type != ScalarType::None; }
};

struct InputLayout {
    FieldDesc x, y, z, intensity, ring, time;
    uint32_t point_step = 0;
    uint32_t row_step   = 0;
    uint32_t width      = 0;
    uint32_t height     = 0;

    bool valid() const {
        if (point_step == 0) return false;
        if (x.type != ScalarType::F32 ||
            y.type != ScalarType::F32 ||
            z.type != ScalarType::F32) return false;
        auto fitsXYZ = [this](const FieldDesc& d) {
            return d.offset >= 0 &&
                   static_cast<uint64_t>(d.offset) + 4 <= point_step;
        };
        return fitsXYZ(x) && fitsXYZ(y) && fitsXYZ(z);
    }
};

template <typename T>
inline T loadAs(const uint8_t* p, int off) {
    T v;
    std::memcpy(&v, p + off, sizeof(T));
    return v;
}

inline float loadAsFloat(const uint8_t* p, const FieldDesc& d) {
    switch (d.type) {
        case ScalarType::I8:  return static_cast<float>(loadAs<int8_t  >(p, d.offset));
        case ScalarType::I16: return static_cast<float>(loadAs<int16_t >(p, d.offset));
        case ScalarType::I32: return static_cast<float>(loadAs<int32_t >(p, d.offset));
        case ScalarType::U8:  return static_cast<float>(loadAs<uint8_t >(p, d.offset));
        case ScalarType::U16: return static_cast<float>(loadAs<uint16_t>(p, d.offset));
        case ScalarType::U32: return static_cast<float>(loadAs<uint32_t>(p, d.offset));
        case ScalarType::F32: return loadAs<float>(p, d.offset);
        case ScalarType::F64: return static_cast<float>(loadAs<double>(p, d.offset));
        default:              return 0.0f;
    }
}

inline double loadAsDouble(const uint8_t* p, const FieldDesc& d) {
    switch (d.type) {
        case ScalarType::I8:  return static_cast<double>(loadAs<int8_t  >(p, d.offset));
        case ScalarType::I16: return static_cast<double>(loadAs<int16_t >(p, d.offset));
        case ScalarType::I32: return static_cast<double>(loadAs<int32_t >(p, d.offset));
        case ScalarType::U8:  return static_cast<double>(loadAs<uint8_t >(p, d.offset));
        case ScalarType::U16: return static_cast<double>(loadAs<uint16_t>(p, d.offset));
        case ScalarType::U32: return static_cast<double>(loadAs<uint32_t>(p, d.offset));
        case ScalarType::F32: return static_cast<double>(loadAs<float>(p, d.offset));
        case ScalarType::F64: return loadAs<double>(p, d.offset);
        default:              return 0.0;
    }
}

inline bool readRing(const uint8_t* p, const FieldDesc& d, uint16_t& out) {
    switch (d.type) {
        case ScalarType::U8:  out = loadAs<uint8_t >(p, d.offset); return true;
        case ScalarType::U16: out = loadAs<uint16_t>(p, d.offset); return true;
        case ScalarType::I8: {
            int8_t v = loadAs<int8_t>(p, d.offset);
            if (v < 0) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        case ScalarType::I16: {
            int16_t v = loadAs<int16_t>(p, d.offset);
            if (v < 0) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        case ScalarType::I32: {
            int32_t v = loadAs<int32_t>(p, d.offset);
            if (v < 0 || v > 65535) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        case ScalarType::U32: {
            uint32_t v = loadAs<uint32_t>(p, d.offset);
            if (v > 65535) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        case ScalarType::F32: {
            float v = loadAs<float>(p, d.offset);
            if (!std::isfinite(v) || v < 0.0f || v > 65535.0f) return false;
            if (std::trunc(v) != v) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        case ScalarType::F64: {
            double v = loadAs<double>(p, d.offset);
            if (!std::isfinite(v) || v < 0.0 || v > 65535.0) return false;
            if (std::trunc(v) != v) return false;
            out = static_cast<uint16_t>(v); return true;
        }
        default: return false;
    }
}

inline void storeF32(uint8_t* p, int off, float v)    { std::memcpy(p + off, &v, 4); }
inline void storeU16(uint8_t* p, int off, uint16_t v) { std::memcpy(p + off, &v, 2); }

inline ScalarType toScalarType(uint8_t dt) {
    using PF = sensor_msgs::msg::PointField;
    switch (dt) {
        case PF::INT8:    return ScalarType::I8;
        case PF::INT16:   return ScalarType::I16;
        case PF::INT32:   return ScalarType::I32;
        case PF::UINT8:   return ScalarType::U8;
        case PF::UINT16:  return ScalarType::U16;
        case PF::UINT32:  return ScalarType::U32;
        case PF::FLOAT32: return ScalarType::F32;
        case PF::FLOAT64: return ScalarType::F64;
        default:          return ScalarType::None;
    }
}

inline double convertTimeUnit(double v, TimeUnit u) {
    switch (u) {
        case TimeUnit::Second:      return v;
        case TimeUnit::Millisecond: return v * 1e-3;
        case TimeUnit::Microsecond: return v * 1e-6;
        case TimeUnit::Nanosecond:  return v * 1e-9;
    }
    return v;
}

using Msg = sensor_msgs::msg::PointCloud2;

struct MsgPool {
    std::mutex mtx;
    std::vector<Msg*> items;
    size_t max_size;
    std::atomic<uint64_t> release_to_delete{0};

    explicit MsgPool(size_t max_sz = 1) : max_size(max_sz) {
        items.reserve(max_sz);
    }
    ~MsgPool() {
        for (auto* m : items) delete m;
    }
    Msg* acquire() {
        std::lock_guard<std::mutex> lk(mtx);
        if (items.empty()) return nullptr;
        Msg* m = items.back();
        items.pop_back();
        return m;
    }
    void release(Msg* m) {
        std::lock_guard<std::mutex> lk(mtx);
        if (items.size() < max_size) items.push_back(m);
        else { ++release_to_delete; delete m; }
    }
};
using MsgPoolPtr = std::shared_ptr<MsgPool>;

struct PoolDeleter {
    MsgPoolPtr pool;
    void operator()(Msg* m) const { if (m) pool->release(m); }
};

using PooledMsg = std::unique_ptr<Msg, PoolDeleter>;

}  // namespace

class RsToVelodyneNode : public rclcpp::Node {
public:
    RsToVelodyneNode() : Node("rs_converter") {
        declare_parameter("input_topic",        "/rslidar_points");
        declare_parameter("output_topic",       "/velodyne_points");
        declare_parameter("output_frame_id",    "");
        declare_parameter("output_type",        "XYZIRT");
        declare_parameter("velodyne_layout",    true);
        declare_parameter("pool_size",          1);
        declare_parameter("max_scan_period",    1.0);

        declare_parameter("ring_source",        "field");
        declare_parameter("ring_count",         16);
        declare_parameter("ring_map",           std::vector<int64_t>{});

        declare_parameter("time_source",        "timestamp");
        declare_parameter("time_mode",          "absolute");
        declare_parameter("time_unit",          "second");
        declare_parameter("time_order_policy",  "validate");

        const auto input_topic  = get_parameter("input_topic").as_string();
        const auto output_topic = get_parameter("output_topic").as_string();
        output_frame_id_        = get_parameter("output_frame_id").as_string();
        velodyne_layout_        = get_parameter("velodyne_layout").as_bool();

        const auto ps = get_parameter("pool_size").as_int();
        if (ps < 1 || ps > kMaxPoolSize) {
            throw std::runtime_error("pool_size must be in [1, " +
                                     std::to_string(kMaxPoolSize) + "], got " +
                                     std::to_string(ps));
        }
        pool_size_ = static_cast<size_t>(ps);

        max_scan_period_ = get_parameter("max_scan_period").as_double();
        if (!std::isfinite(max_scan_period_) || max_scan_period_ <= 0.0) {
            throw std::runtime_error(
                "max_scan_period must be finite and > 0, got " +
                std::to_string(max_scan_period_));
        }

        const auto rs = get_parameter("ring_source").as_string();
        if      (rs == "field")            ring_source_ = RingSource::Field;
        else if (rs == "organized_rows")   ring_source_ = RingSource::OrganizedRows;
        else if (rs == "organized_cols")   ring_source_ = RingSource::OrganizedCols;
        else throw std::runtime_error(
            "ring_source must be 'field' / 'organized_rows' / 'organized_cols', got " + rs);

        const auto rc = get_parameter("ring_count").as_int();
        if (rc < 1 || rc > 65536) {
            throw std::runtime_error(
                "ring_count must be in [1, 65536], got " + std::to_string(rc));
        }
        ring_count_ = rc;

        auto ring_map_raw = get_parameter("ring_map").as_integer_array();
        if (!ring_map_raw.empty()) {
            if (static_cast<int>(ring_map_raw.size()) != ring_count_) {
                throw std::runtime_error(
                    "ring_map size (" + std::to_string(ring_map_raw.size()) +
                    ") must equal ring_count (" + std::to_string(ring_count_) + ")");
            }
            std::vector<bool> seen(ring_count_, false);
            ring_map_.reserve(ring_map_raw.size());
            for (auto v : ring_map_raw) {
                if (v < 0 || v >= ring_count_) {
                    throw std::runtime_error(
                        "ring_map value " + std::to_string(v) +
                        " out of range [0, " + std::to_string(ring_count_) + ")");
                }
                if (seen[v]) {
                    throw std::runtime_error(
                        "ring_map has duplicate value " + std::to_string(v) +
                        "; must be a permutation of [0, " +
                        std::to_string(ring_count_) + ")");
                }
                seen[v] = true;
                ring_map_.push_back(static_cast<uint16_t>(v));
            }
        }

        const auto ts = get_parameter("time_source").as_string();
        if      (ts == "none")       time_source_ = TimeSource::None;
        else if (ts == "timestamp")  time_source_ = TimeSource::Timestamp;
        else if (ts == "time")       time_source_ = TimeSource::Time;
        else throw std::runtime_error(
            "time_source must be 'none' / 'timestamp' / 'time', got " + ts);

        const auto tm = get_parameter("time_mode").as_string();
        if      (tm == "absolute") time_mode_ = TimeMode::Absolute;
        else if (tm == "relative") time_mode_ = TimeMode::Relative;
        else throw std::runtime_error(
            "time_mode must be 'absolute' / 'relative', got " + tm);

        const auto tu = get_parameter("time_unit").as_string();
        if      (tu == "second")      time_unit_ = TimeUnit::Second;
        else if (tu == "millisecond") time_unit_ = TimeUnit::Millisecond;
        else if (tu == "microsecond") time_unit_ = TimeUnit::Microsecond;
        else if (tu == "nanosecond")  time_unit_ = TimeUnit::Nanosecond;
        else throw std::runtime_error(
            "time_unit must be 'second' / 'millisecond' / 'microsecond' / 'nanosecond', got " + tu);

        const auto tp = get_parameter("time_order_policy").as_string();
        if      (tp == "validate") time_order_policy_ = TimeOrderPolicy::Validate;
        else if (tp == "ignore")   time_order_policy_ = TimeOrderPolicy::Ignore;
        else throw std::runtime_error(
            "time_order_policy must be 'validate' / 'ignore', got " + tp);

        const auto t = get_parameter("output_type").as_string();
        if      (t == "XYZI")   out_type_ = OutType::XYZI;
        else if (t == "XYZIR")  out_type_ = OutType::XYZIR;
        else if (t == "XYZIRT") out_type_ = OutType::XYZIRT;
        else throw std::runtime_error("invalid output_type: " + t);

        if (out_type_ == OutType::XYZIRT && time_source_ == TimeSource::None) {
            throw std::runtime_error(
                "output_type=XYZIRT requires time_source != none; "
                "use output_type=XYZIR instead");
        }

        buildOutputLayout();

        pool_ = MsgPoolPtr(new MsgPool(pool_size_));

        auto qos = rclcpp::SensorDataQoS();
        pub_ = create_publisher<Msg>(output_topic, qos);
        sub_ = create_subscription<Msg>(
            input_topic, qos,
            [this](Msg::ConstSharedPtr m) { onCloud(m); });

        RCLCPP_INFO(get_logger(),
                    "rs->velodyne: %s -> %s, out=%s, velodyne_layout=%d, "
                    "ring_source=%s, ring_count=%d, ring_map=%zu entries, "
                    "time_source=%s, time_mode=%s, time_unit=%s, "
                    "time_order_policy=%s, pool_size=%zu, step=%d, "
                    "frame_id='%s', max_scan_period=%.3f",
                    input_topic.c_str(), output_topic.c_str(), t.c_str(),
                    static_cast<int>(use_velodyne_layout_),
                    rs.c_str(), ring_count_, ring_map_.size(),
                    ts.c_str(), tm.c_str(), tu.c_str(), tp.c_str(),
                    pool_size_, out_point_step_,
                    output_frame_id_.empty() ? "(keep input)" : output_frame_id_.c_str(),
                    max_scan_period_);
    }

    ~RsToVelodyneNode() override {
        if (pool_) {
            const auto cnt = pool_->release_to_delete.load();
            RCLCPP_INFO(get_logger(),
                        "pool release_to_delete=%lu (pool_size=%zu)",
                        static_cast<unsigned long>(cnt), pool_size_);
        }
    }

private:
    static constexpr int64_t kMaxPoolSize = 64;

    void buildOutputLayout() {
        out_fields_.clear();
        int off = 0;
        using PF = sensor_msgs::msg::PointField;

        auto addField = [&](const std::string& name, uint8_t dt, int& store_off, int advance) {
            PF f;
            f.name = name;
            f.offset = static_cast<uint32_t>(off);
            f.datatype = dt;
            f.count = 1;
            out_fields_.push_back(f);
            store_off = off;
            off += advance;
        };

        use_velodyne_layout_ = velodyne_layout_ && (out_type_ != OutType::XYZI);

        addField("x", PF::FLOAT32, out_off_x_, 4);
        addField("y", PF::FLOAT32, out_off_y_, 4);
        addField("z", PF::FLOAT32, out_off_z_, 4);

        if (use_velodyne_layout_) off += 4;

        addField("intensity", PF::FLOAT32, out_off_i_, 4);

        if (out_type_ == OutType::XYZIR || out_type_ == OutType::XYZIRT) {
            addField("ring", PF::UINT16, out_off_ring_, 4);
        }
        if (out_type_ == OutType::XYZIRT) {
            addField("time", PF::FLOAT32, out_off_time_, 4);
        }

        if (use_velodyne_layout_) {
            while (off % 32 != 0) off += 4;
        }
        out_point_step_ = off;
    }

    InputLayout parseInputLayout(const Msg& m) const {
        InputLayout L;
        L.point_step = m.point_step;
        L.row_step   = m.row_step;
        L.width      = m.width;
        L.height     = m.height;

        const auto int_max = static_cast<uint32_t>(std::numeric_limits<int>::max());

        for (const auto& f : m.fields) {
            if (f.count != 1) continue;   // 标量字段严格要求 count==1
            if (f.offset > int_max) continue;

            FieldDesc d;
            d.offset = static_cast<int>(f.offset);
            d.type   = toScalarType(f.datatype);

            const int sz = scalarSize(d.type);
            if (sz == 0 ||
                static_cast<uint64_t>(d.offset) + sz > L.point_step) continue;

            if      (f.name == "x")         L.x = d;
            else if (f.name == "y")         L.y = d;
            else if (f.name == "z")         L.z = d;
            else if (f.name == "intensity") L.intensity = d;
            else if (f.name == "ring")      L.ring = d;
            else if (f.name == "timestamp" && time_source_ == TimeSource::Timestamp)
                L.time = d;
            else if (f.name == "time" && time_source_ == TimeSource::Time)
                L.time = d;
        }
        return L;
    }

    PooledMsg acquireMsg(size_t need_bytes) {
        if (Msg* raw = pool_->acquire()) {
            PooledMsg guard(raw, PoolDeleter{pool_});
            if (guard->data.capacity() < need_bytes) {
                guard->data.reserve(need_bytes);
            }
            return guard;
        }

        auto fresh = std::make_unique<Msg>();
        fresh->fields       = out_fields_;
        fresh->point_step   = static_cast<uint32_t>(out_point_step_);
        fresh->is_bigendian = false;
        fresh->data.reserve(need_bytes);

        return PooledMsg(fresh.release(), PoolDeleter{pool_});
    }

    void onCloud(const Msg::ConstSharedPtr& in_msg) {
        if (in_msg->is_bigendian) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "big-endian input not supported, dropping frame");
            return;
        }

        const InputLayout L = parseInputLayout(*in_msg);
        if (!L.valid()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "invalid input layout (x/y/z must be F32, offsets in range)");
            return;
        }

        const uint32_t w32  = L.width;
        const uint32_t h32  = L.height;
        const uint32_t ps32 = L.point_step;
        const uint32_t rs32 = L.row_step;
        if (w32 == 0 || h32 == 0 || ps32 == 0) return;

        if (static_cast<uint64_t>(rs32) !=
            static_cast<uint64_t>(w32) * ps32) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "unsupported row_step: %u != width(%u)*point_step(%u)",
                                 rs32, w32, ps32);
            return;
        }

        const uint64_t total_points = static_cast<uint64_t>(w32) * h32;
        if (total_points == 0 || total_points > kUint32Max) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "invalid total_points: %lu",
                                 static_cast<unsigned long>(total_points));
            return;
        }

        const size_t n = static_cast<size_t>(total_points);

        if (total_points > static_cast<uint64_t>(SIZE_MAX) / ps32) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "input byte size overflow");
            return;
        }
        const size_t need_in_bytes = n * ps32;
        if (in_msg->data.size() < need_in_bytes) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "input data size mismatch: have %zu, need %zu",
                                 in_msg->data.size(), need_in_bytes);
            return;
        }

        if (total_points > static_cast<uint64_t>(SIZE_MAX) /
                           static_cast<uint64_t>(out_point_step_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "output byte size overflow");
            return;
        }
        const uint64_t out_bytes_64 =
            total_points * static_cast<uint64_t>(out_point_step_);
        if (out_bytes_64 > kUint32Max) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "output row_step would exceed UINT32_MAX");
            return;
        }
        const size_t out_bytes = static_cast<size_t>(out_bytes_64);

        const bool want_ring = (out_type_ == OutType::XYZIR || out_type_ == OutType::XYZIRT);
        const bool want_time = (out_type_ == OutType::XYZIRT) &&
                               (time_source_ != TimeSource::None);
        const bool has_ring_in = L.ring.ok();
        const bool has_time_in = L.time.ok();

        if (want_ring && ring_source_ == RingSource::Field && !has_ring_in) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "ring_source=field but input has no ring field; dropping frame");
            return;
        }
        if (want_ring && ring_source_ != RingSource::Field && !(h32 > 1 && w32 > 1)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "ring_source=organized_* requires organized cloud (h=%u, w=%u)",
                                 h32, w32);
            return;
        }
        if (want_time && !has_time_in) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "time_source set but input has no matching field; dropping frame");
            return;
        }

        double base_time = 0.0;
        if (want_time && time_mode_ == TimeMode::Absolute) {
            const auto& stamp = in_msg->header.stamp;
            const bool stamp_valid =
                stamp.nanosec < 1000000000u &&
                stamp.sec >= 0 &&
                (stamp.sec > 0 || stamp.nanosec > 0);
            if (!stamp_valid) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "invalid header.stamp; dropping frame");
                return;
            }
            base_time = static_cast<double>(stamp.sec) +
                        static_cast<double>(stamp.nanosec) * 1e-9;
        }

        PooledMsg guard = acquireMsg(out_bytes);
        Msg* raw = guard.get();

        raw->header.stamp    = in_msg->header.stamp;
        raw->header.frame_id = output_frame_id_.empty()
                               ? in_msg->header.frame_id
                               : output_frame_id_;
        raw->height = 1;
        raw->data.resize(out_bytes);

        const uint8_t* in = in_msg->data.data();
        uint8_t*       out = raw->data.data();
        const size_t   in_step = static_cast<size_t>(L.point_step);
        const uint32_t width  = w32;

        size_t valid = 0;
        uint64_t bad_ring_count = 0;
        uint64_t bad_time_count = 0;

        bool   have_last_time = false;
        double last_time = 0.0;

        for (size_t i = 0; i < n; ++i, in += in_step) {
            const float x = loadAs<float>(in, L.x.offset);
            const float y = loadAs<float>(in, L.y.offset);
            const float z = loadAs<float>(in, L.z.offset);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            uint16_t ring_val = 0;
            bool have_ring = false;
            if (want_ring) {
                uint32_t raw_ring = 0;
                if (ring_source_ == RingSource::Field) {
                    if (!readRing(in, L.ring, ring_val)) {
                        ++bad_ring_count;
                        continue;
                    }
                    raw_ring = ring_val;
                } else if (ring_source_ == RingSource::OrganizedRows) {
                    raw_ring = static_cast<uint32_t>(i) / width;
                } else {  // OrganizedCols
                    raw_ring = static_cast<uint32_t>(i) % width;
                }

                if (raw_ring >= static_cast<uint32_t>(ring_count_)) {
                    ++bad_ring_count;
                    continue;
                }

                if (!ring_map_.empty()) {
                    ring_val = ring_map_[raw_ring];
                } else {
                    ring_val = static_cast<uint16_t>(raw_ring);
                }
                have_ring = true;
            }

            float t = 0.0f;
            if (want_time) {
                const double v = loadAsDouble(in, L.time);
                if (!std::isfinite(v)) {
                    ++bad_time_count;
                    continue;
                }
                const double v_sec = convertTimeUnit(v, time_unit_);
                const double raw_t = (time_mode_ == TimeMode::Absolute)
                                     ? (v_sec - base_time)
                                     : v_sec;

                if (raw_t < -kNegTimeTolerance || raw_t > max_scan_period_) {
                    ++bad_time_count;
                    continue;
                }
                if (time_order_policy_ == TimeOrderPolicy::Validate &&
                    have_last_time && raw_t + kNegTimeTolerance < last_time) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "time regression (%.6f < %.6f); dropping frame",
                                         raw_t, last_time);
                    return;
                }
                if (!have_last_time || raw_t > last_time) {
                    last_time = raw_t;
                }
                have_last_time = true;
                t = static_cast<float>(raw_t < 0.0 ? 0.0 : raw_t);
            }

            uint8_t* p = out + valid * out_point_step_;
            std::memset(p, 0, out_point_step_);
            storeF32(p, out_off_x_, x);
            storeF32(p, out_off_y_, y);
            storeF32(p, out_off_z_, z);

            if (L.intensity.ok()) {
                float intensity = loadAsFloat(in, L.intensity);
                if (!std::isfinite(intensity)) intensity = 0.0f;
                storeF32(p, out_off_i_, intensity);
            }
            if (want_ring && have_ring) {
                storeU16(p, out_off_ring_, ring_val);
            }
            if (want_time) {
                storeF32(p, out_off_time_, t);
            }
            ++valid;
        }

        if (bad_ring_count > 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "dropped %lu points with invalid ring values",
                                 static_cast<unsigned long>(bad_ring_count));
        }
        if (bad_time_count > 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "dropped %lu points with invalid timestamps",
                                 static_cast<unsigned long>(bad_time_count));
            const uint64_t candidates = static_cast<uint64_t>(valid) + bad_time_count;
            if (candidates > 0 && bad_time_count * 10 > candidates) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "invalid timestamp ratio >10%%; dropping frame");
                return;
            }
        }
        if (valid < 2) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "too few valid points (%zu); dropping frame", valid);
            return;
        }

        const size_t valid_bytes = valid * static_cast<size_t>(out_point_step_);
        raw->data.resize(valid_bytes);
        raw->width    = static_cast<uint32_t>(valid);
        raw->row_step = static_cast<uint32_t>(valid_bytes);
        raw->is_dense = true;

        pub_->publish(*guard);
    }

    rclcpp::Subscription<Msg>::SharedPtr sub_;
    rclcpp::Publisher<Msg>::SharedPtr     pub_;

    std::string output_frame_id_;
    OutType     out_type_ = OutType::XYZIRT;
    bool        velodyne_layout_     = true;
    bool        use_velodyne_layout_ = false;
    size_t      pool_size_           = 1;
    double      max_scan_period_     = 1.0;

    RingSource      ring_source_       = RingSource::Field;
    int             ring_count_        = 16;
    std::vector<uint16_t> ring_map_;

    TimeSource      time_source_       = TimeSource::Timestamp;
    TimeMode        time_mode_         = TimeMode::Absolute;
    TimeUnit        time_unit_         = TimeUnit::Second;
    TimeOrderPolicy time_order_policy_ = TimeOrderPolicy::Validate;

    std::vector<sensor_msgs::msg::PointField> out_fields_;
    int out_point_step_ = 0;
    int out_off_x_ = 0, out_off_y_ = 0, out_off_z_ = 0, out_off_i_ = 0;
    int out_off_ring_ = -1, out_off_time_ = -1;

    MsgPoolPtr pool_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RsToVelodyneNode>());
    rclcpp::shutdown();
    return 0;
}
