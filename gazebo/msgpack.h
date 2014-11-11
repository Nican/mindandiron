#include <msgpack_fwd.hpp>
namespace msgpack {

MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {

inline object const& operator>> (object const& o, Eigen::Vector3d& v) {
    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
    if (o.via.array.size != 3) throw msgpack::type_error();

    o.via.array.ptr[0].convert(v.x());
    o.via.array.ptr[1].convert(v.y());
    o.via.array.ptr[2].convert(v.z());

    return o;
}


template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, Eigen::Vector3d const& v) {
    // packing member variables as an array.
    o.pack_array(3);
    o.pack(v.x());
    o.pack(v.y());
    o.pack(v.z());
    return o;
}

} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack

#include <msgpack.hpp>