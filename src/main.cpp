#include "config.hpp"
#include "explain.hpp"

#include <iostream>
#include <memory>

namespace program
{
struct vv_dyn_buf_traits
{
    using store_type   = std::vector< std::vector< char > >;
    using element_type = store_type::value_type;
};

struct vv_dyn_buf_state : vv_dyn_buf_traits
{
    using element_type = std::vector< char >;

    vv_dyn_buf_state(element_type *first, element_type *last)
    : first_element_(first)
    , last_element_(last)
    , initial_discount_(0)
    , final_discount_(0)
    {
    }

    auto build_buffer(std::size_t i) const -> net::mutable_buffer
    {
        if (i >= std::size_t(std::distance(first_element_, last_element_)))
            return net::mutable_buffer();

        auto elem       = first_element_ + i;
        auto first_byte = elem->data();
        auto last_byte  = first_byte + elem->size();
        first_byte += initial_discount(elem);
        last_byte -= final_discount(elem);
        return net::mutable_buffer(first_byte, std::size_t(std::distance(first_byte, last_byte)));
    }

    auto initial_discount(element_type *elem) const -> std::size_t
    {
        if (elem == first_element_ && elem != last_element_)
            return initial_discount_;
        else
            return 0;
    }

    auto final_discount(element_type *elem) const -> std::size_t
    {
        if (elem != last_element_ && std::next(elem) == last_element_)
            return final_discount_;
        else
            return 0;
    }

    auto element_count() const -> std::size_t { return std::size_t(std::distance(first_element_, last_element_)); }

    auto compute_size() const -> std::size_t
    {
        std::size_t total = 0;
        for (auto current = first_element_; current != last_element_; ++current)
        {
            total += current->size() - initial_discount(current) - final_discount(current);
        }
        return total;
    }

    auto available() const -> std::size_t
    {
        if (first_element_ == last_element_)
            return 0;
        else
            return final_discount_;
    }

    void repoint(element_type *first, element_type *last)
    {
        first_element_ = first;
        last_element_  = last;

        rationalise();
    }

    void rationalise()
    {
        if (first_element_ == last_element_)
        {
            initial_discount_ = final_discount_ = 0;
        }
    }

    auto adjust(std::size_t pos, std::size_t n) -> void
    {
        consume(pos, [this] {
            ++first_element_;
            rationalise();
        });
        auto size = compute_size();
        n         = std::min(n, size);
        shrink(size - n, [this] {
            --last_element_;
            rationalise();
        });
    }

    template < class EraseAction >
    auto shrink(std::size_t n, EraseAction on_erase) -> void
    {
        while (n && element_count() != 0)
        {
            auto current = last_element_ - 1;
            auto size    = current->size() - initial_discount(current) - final_discount_;
            auto adj     = std::min(size, n);
            n -= adj;
            final_discount_ += adj;
            if (final_discount_ == current->size() - initial_discount(current))
            {
                final_discount_ = 0;
                on_erase();
            }
        }
    }

    template < class EraseAction >
    auto consume(std::size_t n, EraseAction on_erase) -> void
    {
        while (n && element_count() != 0)
        {
            auto current   = first_element_;
            auto elem_size = current->size() - initial_discount(current) - final_discount(current);
            auto adj       = std::min(elem_size, n);
            n -= adj;
            initial_discount_ += adj;
            if (initial_discount_ == current->size() - final_discount(current))
            {
                initial_discount_ = 0;
                on_erase();
            }
        }
    }

    element_type *first_element_;
    element_type *last_element_;
    std::size_t   initial_discount_ = 0;
    std::size_t   final_discount_   = 0;
};

template < class IsConst >
struct vv_dyn_buf_iterator
{
    struct iterator_category : std::bidirectional_iterator_tag
    {
    };

    using value_type      = std::conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;
    using reference       = value_type const &;
    using pointer         = value_type const *;
    using difference_type = std::ptrdiff_t;

    vv_dyn_buf_iterator(vv_dyn_buf_state const *state = nullptr, std::size_t index = 0)
    : state_(state)
    , index_(index)
    , data_view_()
    {
    }

    template < class IsOtherConst >
    friend struct vv_dyn_buf_iterator;

    value_type operator*() const
    {
        BOOST_ASSERT(state_);
        return state_->build_buffer(index_);
    }

    auto operator++() -> vv_dyn_buf_iterator &
    {
        ++index_;
        return *this;
    }

    auto operator++(int) -> vv_dyn_buf_iterator
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    auto operator--() -> vv_dyn_buf_iterator &
    {
        --index_;
        return *this;
    }

    auto operator--(int) -> vv_dyn_buf_iterator
    {
        auto result = *this;
        --(*this);
        return result;
    }

  private:

    template < class A, class B >
    friend auto operator==(vv_dyn_buf_iterator< A > const &a, vv_dyn_buf_iterator< B > const &b);

    vv_dyn_buf_state const *state_;
    std::size_t             index_ = 0;
    value_type              data_view_;
};

template < class A, class B >
auto operator==(vv_dyn_buf_iterator< A > const &a, vv_dyn_buf_iterator< B > const &b)
{
    return a.index_ == b.index_;
}

template < class A, class B >
auto operator!=(vv_dyn_buf_iterator< A > const &a, vv_dyn_buf_iterator< B > const &b)
{
    return !(a == b);
}

template < class IsConst >
struct vv_dyn_buf_sequence : vv_dyn_buf_traits
{
    using element_type   = std::vector< char >;
    using c_element_type = std::conditional_t< IsConst::value, std::add_const_t< element_type >, element_type >;
    using value_type     = std::conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;

    // we are both an iterator and a sequence
    using iterator       = vv_dyn_buf_iterator< IsConst >;
    using const_iterator = iterator;

    vv_dyn_buf_sequence(vv_dyn_buf_state state)
    : state_(state)
    {
    }

    iterator begin() const { return iterator(std::addressof(state_), 0); }

    iterator end() const { return iterator(std::addressof(state_), state_.element_count()); }

    auto adjust(std::size_t pos, std::size_t n) -> void { state_.adjust(pos, n); }

    vv_dyn_buf_state state_;
};

static_assert(net::is_mutable_buffer_sequence< vv_dyn_buf_sequence< std::false_type > >::value);
static_assert(net::is_const_buffer_sequence< vv_dyn_buf_sequence< std::true_type > >::value);

}   // namespace program

namespace boost
{
namespace asio
{
template < class IsConst >
auto buffer_sequence_begin(program::vv_dyn_buf_sequence< IsConst > const &dyn_buf) ->
    typename program::vv_dyn_buf_sequence< IsConst >::iterator
{
    return dyn_buf.begin();
}

template < class IsConst >
auto buffer_sequence_end(program::vv_dyn_buf_sequence< IsConst > const &dyn_buf) ->
    typename program::vv_dyn_buf_sequence< IsConst >::iterator
{
    return dyn_buf.end();
}

}   // namespace asio
}   // namespace boost

namespace program
{
static_assert(std::is_convertible_v< decltype(boost::asio::buffer_sequence_begin(
                                         std::declval< vv_dyn_buf_sequence< std::false_type > >()))::value_type,
                                     boost::asio::mutable_buffer >);

struct vv_dyn_buf : vv_dyn_buf_traits
{
    vv_dyn_buf(store_type &store, std::size_t max_size = 16 * 1024 * 1024, std::size_t chunk_size = 4096)
    : store_(store)
    , impl_(std::make_shared< vv_dyn_buf_state >(store_.data(), store_.data() + store_.size()))
    , max_size_((std::max)(max_size, impl_->compute_size()))
    , chunk_size_(chunk_size)
    {
    }

    // DynamicBuffer_v2

    using const_buffers_type   = vv_dyn_buf_sequence< std::true_type >;
    using mutable_buffers_type = vv_dyn_buf_sequence< std::false_type >;

    auto size() const -> std::size_t { return impl_->compute_size(); }

    auto max_size() const -> std::size_t { return max_size_; }

    auto capacity() const -> std::size_t { return impl_->compute_size() + impl_->available(); }

    operator const_buffers_type() const { return const_buffers_type(*impl_); }

    operator mutable_buffers_type() const { return mutable_buffers_type(*impl_); }

    auto data(std::size_t pos, std::size_t n) const -> const_buffers_type
    {
        auto result = const_buffers_type(*this);
        result.adjust(pos, n);
        return result;
    }

    auto data(std::size_t pos, std::size_t n) -> mutable_buffers_type
    {
        auto result = mutable_buffers_type(*this);
        result.adjust(pos, n);
        return result;
    }

    void repoint() { impl_->repoint(store_.data(), store_.data() + store_.size()); }

    auto grow(std::size_t n) -> void
    {
        if (n + size() > max_size())
            throw std::length_error("grow");

        auto avail = std::min(impl_->available(), n);
        impl_->final_discount_ -= avail;
        n -= avail;

        // need to allocate another block?
        if (n)
        {
            auto chunk_size = std::max(std::size_t(chunk_size_), n);
            store_.emplace_back(chunk_size);
            repoint();
            impl_->final_discount_ = chunk_size - n;
        }
    }

    auto shrink(std::size_t n) -> void
    {
        impl_->shrink(n, [this] {
            store_.pop_back();
            repoint();
        });
    }

    auto consume(std::size_t n) -> void
    {
        impl_->consume(n, [this] {
            store_.erase(store_.begin());
            repoint();
        });
    }

  private:
    store_type &                        store_;
    std::shared_ptr< vv_dyn_buf_state > impl_;
    const std::size_t                   max_size_;
    const std::size_t                   chunk_size_;
};

static_assert(net::is_dynamic_buffer_v2< vv_dyn_buf >::value);

auto dynamic_buffer(std::vector< std::vector< char > > &store)
{
    return vv_dyn_buf(store);
}
}   // namespace program

namespace program
{
namespace websocket = beast::websocket;
using tcp           = net::ip::tcp;

using namespace std::literals;

auto build_buffers_storage(const char *const *strings) -> std::vector< std::vector< char > >
{
    std::vector< std::vector< char > > result;

    while (auto string = *strings++)
    {
        auto len = std::strlen(string);
        result.emplace_back();
        auto &vec = result.back();
        vec.reserve(len + 2);
        std::copy(string, string + len, std::back_inserter(vec));
        vec.push_back('\r');
        vec.push_back('\n');
    }

    return result;
}

int run()
{
    auto            host = "www.example.com"s, endpoint = "/"s, port = "80"s;
    net::io_context ioc;
    auto            exec = ioc.get_executor();

    tcp::resolver resolver { exec };

    const auto resolved = resolver.resolve(host, port);
    auto       sock     = tcp::socket(exec);

    boost::asio::connect(sock, resolved);

    const char *const request_texts[] = {
        "GET / HTTP/1.1", "Host: example.com", "User-Agent: curl/7.66.0", "Accept: */*", "", nullptr
    };

    auto storage = build_buffers_storage(request_texts);
    {
        const auto wb = dynamic_buffer(storage);
        net::write(sock, wb.data(0, wb.size()));
        sock.shutdown(tcp::socket::shutdown_send);
    }

    storage.clear();
    auto readbuf    = dynamic_buffer(storage);
    auto ec         = system::error_code();
    auto bytes_read = net::read_until(sock, readbuf, net::string_view("\r\n\r\n"), ec);
    std::cout << "read size: " << bytes_read << std::endl;
    std::cout << "buffer size: " << readbuf.size() << std::endl;
    std::cout << "Headers:\n" << beast::buffers_to_string(readbuf.data(0, bytes_read));
    readbuf.consume(bytes_read);
    bytes_read = net::read(sock, readbuf, ec);
    std::cout << "read size: " << bytes_read << std::endl;
    std::cout << "buffer size: " << readbuf.size() << std::endl;
    auto as_str = beast::buffers_to_string(readbuf.data(0, readbuf.size()));
    std::cout << "string size: " << as_str.size() << std::endl;
    std::cout << "Body:\n" << as_str << std::endl;
    readbuf.consume(bytes_read);

    sock.shutdown(tcp::socket::shutdown_receive, ec);
    sock.close();

    return 0;
}

}   // namespace program

int main()
{
    try
    {
        return program::run();
    }
    catch (...)
    {
        std::cerr << program::explain() << std::endl;
        return 127;
    }
}