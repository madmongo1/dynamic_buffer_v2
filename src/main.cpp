#include "config.hpp"
#include "explain.hpp"

#include <iostream>
#include <memory>
#include <list>
#include <vector>
#include <deque>

namespace program
{
template < class Iterator >
struct cobc_dynamic_buffer_state
{
    using value_type = typename Iterator::value_type;

    cobc_dynamic_buffer_state(Iterator first, Iterator last)
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

        auto elem       = std::next(first_element_, i);
        auto first_byte = elem->data();
        auto last_byte  = first_byte + elem->size();
        first_byte += initial_discount(elem);
        last_byte -= final_discount(elem);
        return net::mutable_buffer(first_byte, std::size_t(std::distance(first_byte, last_byte)));
    }

    auto initial_discount(Iterator elem) const -> std::size_t
    {
        if (elem == first_element_ && elem != last_element_)
            return initial_discount_;
        else
            return 0;
    }

    auto final_discount(Iterator elem) const -> std::size_t
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

    void repoint(Iterator first, Iterator last)
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
            auto current = std::prev(last_element_);
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

    Iterator    first_element_;
    Iterator    last_element_;
    std::size_t initial_discount_ = 0;
    std::size_t final_discount_   = 0;
};

template < class StorageIterator, class IsConst >
struct cobc_dynamic_buffer_iterator
{
    struct iterator_category : std::bidirectional_iterator_tag
    {
    };

    using value_type      = std::conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;
    using reference       = value_type const &;
    using pointer         = value_type const *;
    using difference_type = std::ptrdiff_t;
    using state_type      = cobc_dynamic_buffer_state< StorageIterator >;

    cobc_dynamic_buffer_iterator(state_type const *state = nullptr, std::size_t index = 0)
    : state_(state)
    , index_(index)
    , data_view_()
    {
    }

    template < class OtherStorageIterator, class IsOtherConst >
    friend struct cobc_dynamic_buffer_iterator;

    value_type operator*() const
    {
        BOOST_ASSERT(state_);
        return state_->build_buffer(index_);
    }

    auto operator++() -> cobc_dynamic_buffer_iterator &
    {
        ++index_;
        return *this;
    }

    auto operator++(int) -> cobc_dynamic_buffer_iterator
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    auto operator--() -> cobc_dynamic_buffer_iterator &
    {
        --index_;
        return *this;
    }

    auto operator--(int) -> cobc_dynamic_buffer_iterator
    {
        auto result = *this;
        --(*this);
        return result;
    }

  private:
    template < class AI, class AB, class BI, class BB >
    friend auto operator==(cobc_dynamic_buffer_iterator< AI, AB > const &a,
                           cobc_dynamic_buffer_iterator< BI, BB > const &b);

    state_type const *state_;
    std::size_t       index_ = 0;
    value_type        data_view_;
};

template < class AI, class AB, class BI, class BB >
auto operator==(cobc_dynamic_buffer_iterator< AI, AB > const &a, cobc_dynamic_buffer_iterator< BI, BB > const &b)
{
    return a.index_ == b.index_;
}

template < class AI, class AB, class BI, class BB >
auto operator!=(cobc_dynamic_buffer_iterator< AI, AB > const &a, cobc_dynamic_buffer_iterator< BI, BB > const &b)
{
    return !(a == b);
}

template < class StoreIterator, class IsConst >
struct cobc_dynamic_buffer_sequence
{
    using element_type   = typename StoreIterator::value_type;
    using c_element_type = std::conditional_t< IsConst::value, std::add_const_t< element_type >, element_type >;
    using value_type     = std::conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;

    // we are both an iterator and a sequence
    using iterator       = cobc_dynamic_buffer_iterator< StoreIterator, IsConst >;
    using const_iterator = iterator;

    cobc_dynamic_buffer_sequence(cobc_dynamic_buffer_state< StoreIterator > state)
    : state_(state)
    {
    }

    iterator begin() const { return iterator(std::addressof(state_), 0); }

    iterator end() const { return iterator(std::addressof(state_), state_.element_count()); }

    auto adjust(std::size_t pos, std::size_t n) -> void { state_.adjust(pos, n); }

    cobc_dynamic_buffer_state< StoreIterator > state_;
};

static_assert(net::is_mutable_buffer_sequence<
              cobc_dynamic_buffer_sequence< std::vector< std::vector< char > >, std::false_type > >::value);

static_assert(
    net::is_const_buffer_sequence<
              cobc_dynamic_buffer_sequence< std::vector< std::vector< char > >, std::true_type > >::value);

static_assert(
    std::is_convertible_v<
        decltype(boost::asio::buffer_sequence_begin(
            std::declval< cobc_dynamic_buffer_sequence< std::vector< std::vector< char > >, std::false_type > >()))::value_type,
        boost::asio::mutable_buffer >);

template < class StoreType >
struct cobc_dynamic_buffer
{
    using store_type = StoreType;
    using state_type = cobc_dynamic_buffer_state< typename store_type::iterator >;

    cobc_dynamic_buffer(store_type &store, std::size_t max_size = 16 * 1024 * 1024, std::size_t chunk_size = 4096)
    : store_(store)
    , impl_(std::make_shared< state_type >(store_.begin(), store_.end()))
    , max_size_((std::max)(max_size, impl_->compute_size()))
    , chunk_size_(chunk_size)
    {
    }

    // DynamicBuffer_v2

    using const_buffers_type   = cobc_dynamic_buffer_sequence< typename StoreType::iterator, std::true_type >;
    using mutable_buffers_type = cobc_dynamic_buffer_sequence< typename StoreType::iterator, std::false_type >;

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

    void repoint() { impl_->repoint(store_.begin(), store_.end()); }

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
    store_type &                  store_;
    std::shared_ptr< state_type > impl_;
    const std::size_t             max_size_;
    const std::size_t             chunk_size_;
};

static_assert(net::is_dynamic_buffer_v2< cobc_dynamic_buffer< std::vector< std::vector< char > > > >::value);

template < class Container, typename = void >
struct is_container_of_bytes : std::false_type
{
};

template < class Container >
struct is_container_of_bytes<
    Container,
    std::enable_if_t< sizeof(std::decay_t< decltype(*std::begin(std::declval< Container >())) >) == 1 > >
: std::true_type
{
};

template < class Container, typename = void >
struct is_container_of_container_of_bytes : std::false_type
{
};

template < class Container >
struct is_container_of_container_of_bytes<
    Container,
    std::enable_if_t< is_container_of_bytes< decltype(*std::begin(std::declval< Container >())) >::value > >
: std::true_type
{
};

static_assert(is_container_of_bytes< std::vector< char > >::value);
static_assert(is_container_of_container_of_bytes< std::vector< std::vector< char > > >::value);

template < class Container >
auto dynamic_buffer(Container &store)
    -> std::enable_if_t< is_container_of_container_of_bytes< Container >::value, cobc_dynamic_buffer< Container > >
{
    return cobc_dynamic_buffer< Container >(store);
}
}   // namespace program

namespace program
{
namespace websocket = beast::websocket;
using tcp           = net::ip::tcp;

using namespace std::literals;

template < class StoreType >
auto build_buffers_storage(const char *const *strings) -> StoreType
{
    StoreType result;

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

template < class StoreType >
void check_your_privilege(net::executor exec, tcp::resolver::results_type resolved)
{
    auto sock = tcp::socket(exec);

    boost::asio::connect(sock, resolved);

    const char *const request_texts[] = {
        "GET / HTTP/1.1", "Host: example.com", "User-Agent: curl/7.66.0", "Accept: */*", "", nullptr
    };

    auto storage = build_buffers_storage< StoreType >(request_texts);
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
}

int run()
{
    auto            host = "www.example.com"s, endpoint = "/"s, port = "80"s;
    net::io_context ioc;
    auto            exec = ioc.get_executor();

    tcp::resolver resolver { exec };

    const auto resolved = resolver.resolve(host, port);
    check_your_privilege< std::vector< std::vector< char > > >(exec, resolved);
    check_your_privilege< std::deque< std::vector< char > > >(exec, resolved);
    check_your_privilege< std::list< std::vector< char > > >(exec, resolved);

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