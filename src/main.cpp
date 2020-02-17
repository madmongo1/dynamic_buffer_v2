#include "config.hpp"
#include "explain.hpp"

#include <boost/core/typeinfo.hpp>
#include <deque>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <vector>

namespace program
{
namespace detail
{
template < class Container, typename = void >
struct is_container_of_bytes : std::false_type
{
};

template < class Container >
struct is_container_of_bytes<
    Container,
    std::enable_if_t< sizeof(std::decay_t< decltype(
                                 *(std::declval< Container >().data())) >) ==
                      1 > > : std::true_type
{
};

static_assert(is_container_of_bytes< std::vector< char > >::value);
static_assert(!is_container_of_bytes< std::list< char > >::value);

template < class Container, typename = void >
struct is_container_of_container_of_bytes : std::false_type
{
};

template < class Container >
struct is_container_of_container_of_bytes<
    Container,
    std::enable_if_t< is_container_of_bytes< decltype(
        *std::begin(std::declval< Container >())) >::value > > : std::true_type
{
};

static_assert(is_container_of_container_of_bytes<
              std::vector< std::vector< char > > >::value);
}   // namespace detail

/// Hold the current state of either a dynamic buffer or a dynamic_buffer
/// subsequence.
/// \tparam Iterator A type of iterator who's value type will be a
///                  contiguous byte container
/// \note This class could be further refactored into 2 - state and dyn_buffer
/// state, which could then maintain a chached size_. This would speed up
/// grow() operations a teeny bit because we could track the size rather than
/// having to compute it
template < class Iterator >
struct cobc_state
{
    /// The type of sequence of bytes being pointed to by the Iterator
    using value_type = typename std::iterator_traits< Iterator >::value_type;
    static_assert(detail::is_container_of_bytes< value_type >::value);

    /// Constructor.
    ///
    /// Construct a state representing the full extent of bytes bounded by
    /// two iterators, first and last
    /// \param first
    /// \param last
    cobc_state(Iterator first, Iterator last)
    : first_element_(first)
    , last_element_(last)
    , initial_discount_(0)
    , final_discount_(0)
    {
    }

    auto first_element() const -> Iterator { return first_element_; }

    auto last_element() const -> Iterator { return last_element_; }

    /// Given an element iterator, return a net::mutable_buffer representing the
    /// addressable bytes held in element *std::next(first_element_, i)
    /// modulo the conditional discounts
    /// \param elem
    /// \return net::mutable_buffer
    auto build_buffer(Iterator elem) const -> net::mutable_buffer
    {
        auto first_byte = elem->data();
        auto last_byte  = first_byte + elem->size() - final_discount(elem);
        first_byte += initial_discount(elem);
        return net::mutable_buffer(
            first_byte, std::size_t(std::distance(first_byte, last_byte)));
    }

    /// Return the initial discount for a given iterator
    /// \param elem
    /// \return std::size_t
    auto initial_discount(Iterator elem) const -> std::size_t
    {
        if (elem == first_element_ && elem != last_element_)
            return initial_discount_;
        else
            return 0;
    }

    /// Return the final discount for a given iterator
    /// \param elem
    /// \return std::size_t
    auto final_discount(Iterator elem) const -> std::size_t
    {
        if (elem != last_element_ && std::next(elem) == last_element_)
            return final_discount_;
        else
            return 0;
    }

    /// Return the number of regions bounded by a subrange controlled by this
    /// state
    /// \return std::size_t
    auto element_count() const -> std::size_t
    {
        return std::size_t(std::distance(first_element_, last_element_));
    }

    /// Compute the total number of bytes addressed by a subrange controlled by
    /// this state
    /// \return std::size_t
    auto compute_size() const -> std::size_t
    {
        std::size_t total = 0;
        for (auto current = first_element_; current != last_element_; ++current)
        {
            total += current->size() - initial_discount(current) -
                     final_discount(current);
        }
        return total;
    }

    /// Return the number of bytes by which the subrange can grow without
    /// requiring more storage
    /// \return std::size_t
    auto available() const -> std::size_t
    {
        if (first_element_ == last_element_)
            return 0;
        else
            return final_discount_;
    }

    /// Reset the first and last iterators.
    ///
    /// For example, called after storage acquired or destroyed
    /// \param first
    /// \param last
    void repoint(Iterator first, Iterator last)
    {
        first_element_ = first;
        last_element_  = last;

        rationalise();
    }

    /// Ensure that the discounts are zero if the subrange has no elements
    void rationalise()
    {
        if (first_element_ == last_element_)
        {
            initial_discount_ = final_discount_ = 0;
        }
    }

    /// Reduce the size of the subrange
    /// \param pos discard all bytes before this position
    /// \param n discard all bytes after this extent
    auto adjust(std::size_t pos, std::size_t n) -> void
    {
        consume(pos, [this] {
            first_element_ = std::next(first_element_);
            rationalise();
        });
        auto size = compute_size();
        n         = std::min(n, size);
        shrink(size - n, [this] {
            last_element_ = std::prev(last_element_);
            rationalise();
        });
    }

    /// Remove visibility of bytes from the end of the subrange
    /// \tparam EraseAction
    /// \param n the number of bytes by which to shrink
    /// \param on_erase function to call whenever a storage element needs to be
    ///                 removed or destroyed from the end of this state
    template < class EraseAction >
    auto shrink(std::size_t n, EraseAction on_erase) -> void
    {
        while (n && element_count() != 0)
        {
            auto current = std::prev(last_element_);
            auto size =
                current->size() - initial_discount(current) - final_discount_;
            auto adj = std::min(size, n);
            n -= adj;
            final_discount_ += adj;
            if (final_discount_ == current->size() - initial_discount(current))
            {
                final_discount_ = 0;
                on_erase();
            }
        }
    }

    /// Remove visibility of bytes from the start of the subrange
    /// \tparam EraseAction
    /// \param n the number of bytes by which to shrink
    /// \param on_erase function to call whenever a storage element needs to be
    ///                 removed or destroyed from the front of this state
    template < class EraseAction >
    auto consume(std::size_t n, EraseAction on_erase) -> void
    {
        while (n && element_count() != 0)
        {
            auto current   = first_element_;
            auto elem_size = current->size() - initial_discount(current) -
                             final_discount(current);
            auto adj = std::min(elem_size, n);
            n -= adj;
            initial_discount_ += adj;
            if (initial_discount_ == current->size() - final_discount(current))
            {
                initial_discount_ = 0;
                on_erase();
            }
        }
    }

    template < class AddChunk >
    void grow(std::size_t n, AddChunk add_chunk)
    {
        auto avail = std::min(available(), n);
        final_discount_ -= avail;
        n -= avail;

        // need to allocate another block?
        while (n)
        {
            auto chunk_size = add_chunk(n);
            if (chunk_size >= n)
            {
                final_discount_ = chunk_size - n;
                n               = 0;
            }
            else
            {
                n -= chunk_size;
            }
        }
    }

  private:
    Iterator    first_element_;
    Iterator    last_element_;
    std::size_t initial_discount_;
    std::size_t final_discount_;
};

/// An bidrectional (optionally random access) iterator which produces either a
/// mutable_buffer or a const_buffer.
/// \tparam StorageIterator An iterator into a container of storage regions
/// \tparam IsConst either std::true_type or std::false_type
template < class StorageIterator, class IsConst >
struct cobc_iterator
{
    // for now we only implement bidrectional. this could be borrowed from
    // StorageIterator
    struct iterator_category : std::bidirectional_iterator_tag
    {
    };

    using value_type = std::
        conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;
    using reference       = value_type const &;
    using pointer         = value_type const *;
    using difference_type = std::ptrdiff_t;
    using state_type      = cobc_state< StorageIterator >;

    /// Construct at a specific index in a specific state
    /// \param state optional pointer to state which describes the subrange
    /// \param index current element position may be 0 to state->element_count()
    /// \note also serves as default constructor in order to satisfy
    ///       DefaultConstructible
    explicit cobc_iterator()
    : state_(nullptr)
    , index_(StorageIterator())
    {
    }

    explicit cobc_iterator(state_type const *state, StorageIterator index)
    : state_(state)
    , index_(index)
    {
    }

    // so we can compare const_iterators with iterators
    template < class OtherStorageIterator, class IsOtherConst >
    friend struct cobc_iterator;

    /// Yield a Buffer at the current position
    /// \return net::const_buffer or net::mutable_buffer
    /// \pre must be pointing at a valid element in state
    value_type operator*() const
    {
        BOOST_ASSERT(state_);
        return state_->build_buffer(index_);
    }

    /// Pre-increment
    auto operator++() -> cobc_iterator &
    {
        index_ = std::next(index_);
        return *this;
    }

    /// Post-increment
    auto operator++(int) -> cobc_iterator
    {
        auto result = *this;
        ++(*this);
        return result;
    }

    /// Pre-decrement
    auto operator--() -> cobc_iterator &
    {
        index_ = std::prev(index_);
        return *this;
    }

    /// Post-decrement
    auto operator--(int) -> cobc_iterator
    {
        auto result = *this;
        --(*this);
        return result;
    }

  private:
    template < class AI, class AB, class BI, class BB >
    friend auto operator==(cobc_iterator< AI, AB > const &a,
                           cobc_iterator< BI, BB > const &b);

    state_type const *state_;
    StorageIterator   index_;
};

template < class AI, class AB, class BI, class BB >
auto operator==(cobc_iterator< AI, AB > const &a,
                cobc_iterator< BI, BB > const &b)
{
    return a.index_ == b.index_;
}

template < class AI, class AB, class BI, class BB >
auto operator!=(cobc_iterator< AI, AB > const &a,
                cobc_iterator< BI, BB > const &b)
{
    return !(a == b);
}

/// Describe a subrange of a dynamic_buffer
/// \tparam StoreIterator is an iterator who's value type yields a container of
///                       contiguous bytes
/// \tparam IsConst
template < class StoreIterator, class IsConst >
struct cobc_subrange
{
    using element_type =
        typename std::iterator_traits< StoreIterator >::value_type;
    static_assert(detail::is_container_of_bytes< element_type >::value);
    using c_element_type = std::conditional_t< IsConst::value,
                                               std::add_const_t< element_type >,
                                               element_type >;
    using value_type     = std::
        conditional_t< IsConst::value, net::const_buffer, net::mutable_buffer >;

    // we are both an iterator and a sequence
    using iterator       = cobc_iterator< StoreIterator, IsConst >;
    using const_iterator = iterator;

    cobc_subrange(cobc_state< StoreIterator > state)
    : state_(state)
    {
    }

    iterator begin() const
    {
        return iterator(std::addressof(state_), state_.first_element());
    }

    iterator end() const
    {
        return iterator(std::addressof(state_), state_.last_element());
    }

    auto adjust(std::size_t pos, std::size_t n) -> void
    {
        state_.adjust(pos, n);
    }

    cobc_state< StoreIterator > state_;
};

static_assert(net::is_mutable_buffer_sequence<
              cobc_subrange< std::vector< std::vector< char > >::iterator,
                             std::false_type > >::value);

static_assert(net::is_const_buffer_sequence<
              cobc_subrange< std::vector< std::vector< char > >::iterator,
                             std::true_type > >::value);

static_assert(
    std::is_convertible_v<
        std::iterator_traits< decltype(boost::asio::buffer_sequence_begin(
            std::declval<
                cobc_subrange< std::vector< std::vector< char > >::iterator,
                               std::false_type > >())) >::value_type,
        boost::asio::mutable_buffer >);

template < class StoreType >
struct cobc_dynamic_buffer
{
    using store_type = StoreType;
    using state_type = cobc_state< typename store_type::iterator >;

    cobc_dynamic_buffer(store_type &store,
                        std::size_t max_size   = 16 * 1024 * 1024,
                        std::size_t chunk_size = 4096)
    : store_(store)
    , impl_(std::make_shared< state_type >(store_.begin(), store_.end()))
    , max_size_((std::max)(max_size, impl_->compute_size()))
    , chunk_size_(chunk_size)
    {
    }

    // DynamicBuffer_v2

    using const_buffers_type =
        cobc_subrange< typename StoreType::iterator, std::true_type >;
    using mutable_buffers_type =
        cobc_subrange< typename StoreType::iterator, std::false_type >;

    auto size() const -> std::size_t { return impl_->compute_size(); }

    auto max_size() const -> std::size_t { return max_size_; }

    auto capacity() const -> std::size_t
    {
        return impl_->compute_size() + impl_->available();
    }

    operator const_buffers_type() const { return const_buffers_type(*impl_); }

    operator mutable_buffers_type() const
    {
        return mutable_buffers_type(*impl_);
    }

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

        impl_->grow(n, [this](std::size_t required) {
            auto chunk_size = std::max(std::size_t(chunk_size_), required);
            store_.emplace_back(chunk_size);
            repoint();
            return chunk_size;
        });
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

static_assert(
    net::is_dynamic_buffer_v2<
        cobc_dynamic_buffer< std::vector< std::vector< char > > > >::value);

template < class Container >
auto dynamic_buffer(Container & store,
                    std::size_t max_size   = 16 * 1024 * 1024,
                    std::size_t chunk_size = 4096)
    -> std::enable_if_t<
        detail::is_container_of_container_of_bytes< Container >::value,
        cobc_dynamic_buffer< Container > >
{
    return cobc_dynamic_buffer< Container >(store, max_size, chunk_size);
}

// =============
// start of test
// =============

using tcp = net::ip::tcp;
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
void check_your_privilege(net::executor               exec,
                          tcp::resolver::results_type resolved,
                          StoreType                   storage)
try
{
    auto sock = tcp::socket(exec);

    boost::asio::connect(sock, resolved);

    const auto wb = dynamic_buffer(storage);
    net::write(sock, wb.data(0, wb.size()));
    sock.shutdown(tcp::socket::shutdown_send);

    storage.clear();
    auto readbuf = dynamic_buffer(storage);
    auto ec      = system::error_code();
    auto bytes_read =
        net::read_until(sock, readbuf, net::string_view("\r\n\r\n"), ec);
    std::cout << "read size: " << bytes_read << std::endl;
    std::cout << "buffer size: " << readbuf.size() << std::endl;
    std::cout << "Headers:\n"
              << beast::buffers_to_string(readbuf.data(0, bytes_read));
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
catch (...)
{
    std::throw_with_nested(
        std::runtime_error("failed check with type : "s +
                           boost::core::demangled_name(typeid(StoreType))));
}

int run()
{
    auto            host = "www.example.com"s, endpoint = "/"s, port = "80"s;
    net::io_context ioc;
    auto            exec = ioc.get_executor();

    tcp::resolver resolver { exec };

    const auto resolved = resolver.resolve(host, port);

    const char *const request_texts[] = { "GET / HTTP/1.1",
                                          "Host: example.com",
                                          "User-Agent: curl/7.66.0",
                                          "Accept: */*",
                                          "",
                                          nullptr };

    check_your_privilege(
        exec,
        resolved,
        build_buffers_storage< std::vector< std::vector< char > > >(
            request_texts));

    check_your_privilege(
        exec,
        resolved,
        build_buffers_storage< std::deque< std::vector< char > > >(
            request_texts));

    check_your_privilege(
        exec,
        resolved,
        build_buffers_storage< std::list< std::vector< char > > >(
            request_texts));

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