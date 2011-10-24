#ifndef RB_TREE_H_
#define RB_TREE_H_

#include <functional>
#include <iterator>
#include <assert.h>


enum {	RED, BLACK	} ;

template <typename T>
struct node {
    static  node*          SENTINEL;
    typedef typename T	   value_type;
    typedef unsigned long  size_type;
    T     m_value;
    node* m_parent;
    node* m_left;
    node* m_right;
    bool  m_color;
    node(value_type in_value, node* in_parent, bool in_color ): 
    m_value  (in_value), 
        m_parent (in_parent),  
        m_color  (in_color), 
        m_left   (SENTINEL), 
        m_right  (SENTINEL)
    {}
    node(value_type in_value, node* in_parent ): m_value  (in_value), 
        m_parent (in_parent),  
        m_color  (BLACK),   
        m_left   (SENTINEL), 
        m_right  (SENTINEL)	
    {}
    explicit node(const value_type& in_value ): node   (in_value), 
        m_color  (BLACK),   
        m_parent (SENTINEL), 
        m_left   (SENTINEL), 
        m_right  (SENTINEL) 
    {}
    node():  m_color (BLACK),  
        m_parent (SENTINEL), 
        m_left   (SENTINEL), 
        m_right  (SENTINEL) 
    {}
    node(node const& rhs )	{	
        (*this ) = rhs ;	
    }
    node* sibling(void )	{	
        return (is_left_son() ) ? m_parent->m_right : m_parent->m_left ;	
    }
    value_type& operator =(const value_type& in_value ) {
        m_value = in_value; 
    }
    bool operator == (const value_type& in_value ) const {
        return m_value == in_value ;	
    }
    bool operator != (const value_type& in_value ) const {	
        return m_value != in_value ;	
    }
    bool operator > (const value_type& in_value )  const {	
        return m_value > in_value ;	
    }
    bool operator<(const value_type& in_value )	  const {	
        return m_value < in_value ;	
    }
    bool operator >= (const value_type& in_value ) const {	
        return m_value >= in_value ;	
    }
    bool operator <= (const value_type& in_value ) const {	
        return m_value <= in_value ;	
    }
    bool operator == (const node& rhs ) const {	
        return (m_value == rhs.m_value ) && (m_parent == rhs.m_parent ) ;	
    }
    bool operator!= (const node& rhs ) const {	
        return (m_value != rhs.m_value ) || (m_parent != rhs.m_parent ) ;	
    }
    bool operator > (const node& rhs ) const {	
        return m_value > rhs.m_value ;	
    }
    bool operator < (const node& rhs ) const {	
        return m_value < rhs.m_value ;	
    }
    bool operator >=(const node& rhs ) const	{	
        return m_value >= rhs.m_value ;	
    }
    bool operator <= (const node& rhs ) const	{	
        return m_value <= other.m_value ;	
    }
    const bool is_left_son(void ) const	{	
        return (m_parent != SENTINEL ) && (m_parent->m_left == this ) ;		
    }
    const bool is_right_son(void ) const	{	
        return (m_parent != SENTINEL ) && (m_parent->m_right == this ) ;	
    }
    node* maximum()	{	
        return (m_right == SENTINEL ) ? this : m_right->maximum() ;	
    }
    node* minimum()	{	
        return (m_left == SENTINEL ) ? this : m_left->minimum() ;	
    }
    node* get_successor() {
        if (m_right != SENTINEL )
            return m_right->minimum() ;
        if (is_left_son() )
            return m_parent ;
        node* succ = this ;
        do { 
            succ = succ->m_parent ; 
        }
        while ((succ != SENTINEL ) && succ->is_right_son() ) ;
        if (succ != SENTINEL )
            return succ->m_parent ;
        else
            return SENTINEL ;
    }
    node* get_predecessor() {
        if (m_left != SENTINEL )
            return m_left->maximum() ;
        if (is_right_son() )
            return m_parent ;
        node* pred = this ;
        do { 
            pred = pred->m_parent ; 
        }
        while ((pred != SENTINEL ) && pred->is_left_son() ) ;
        if (pred != SENTINEL )
            return pred->m_parent ;
        else
            return SENTINEL ;
    }
private:
    static node SENTINEL_OBJECT ;
};
template< typename T >
node<T> node<T>::SENTINEL_OBJECT =  node<T>() ;

template< typename T >
node<T> * node<T>::SENTINEL = &node<T>::SENTINEL_OBJECT ;

template< typename T, typename CMP = std::less<T> >
class tree {
public:
    typedef typename node<T> node_type       ;
    typedef typename T            value_type      ;
    typedef value_type&								reference       ;
    typedef value_type const &							const_reference ;
    typedef typename node_type::size_type				size_type       ;

    static node_type*	SENTINEL ;

    template< typename T >
    class __iterator: public std::iterator< std::bidirectional_iterator_tag,
        typename T::value_type,
        typename T::size_type	> {
    private:
        T *m_iterator;
    public:
        typedef typename T::value_type *			pointer ;
        typedef typename T::value_type const *		const_pointer ;
        typedef typename T::value_type &			reference ;
        typedef typename T::value_type const &		const_reference ;
        explicit __iterator(void ): std::iterator(), 
            m_iterator(T::SENTINEL ) { } ;
        __iterator(T* ptr ): m_iterator(ptr ) { }
        __iterator(const __iterator& rhs ): m_iterator(rhs.m_iterator ) { }

        __iterator operator++(void ) {
            m_iterator = m_iterator->successor() ;
            return* this ;
        }
        __iterator operator++(int ) {
            T* temp = m_iterator ;
            operator++() ;
            return __iterator(temp ) ;
        }
        __iterator operator--(void ) {
            m_iterator = m_iterator->predecessor() ;
            return (*this ) ;
        }
        __iterator operator--(int ) {
            T* temp = m_iterator ;
            operator--() ;
            return m_iterator(temp ) ;
        }
        __iterator& operator=(const_reference rhs ) {
            m_iterator->in_value = rhs ;
            return (*this );
        }
        bool operator==(const __iterator& rhs ) const {	
            return (m_iterator == rhs.m_iterator ) ;	
        }
        bool operator!=(const __iterator& rhs ) const {	
            return (m_iterator != rhs.m_iterator ) ;	
        }
        operator T&(){	
            return *m_iterator ;	
        }
        operator const T&() const	{	
            return *m_iterator ;	
        }
        reference operator* (void )	{	
            return m_iterator->in_value ;		
        }
        T* operator->(void ){	
            return m_iterator ;	
        }
    };

    typedef typename __iterator<node_type>   iterator ;
    typedef typename iterator const          const_iterator ;

    tree(): m_root(SENTINEL),  m_size(0)	{}

    iterator  begin()const {   
        return minimum();
    }
    iterator end() const {   
        return iterator();	       
    }
    iterator root() const {   
        return iterator(m_root );  
    }
    bool empty() const {   
        return (m_root == SENTINEL );   
    }
    size_type size() const {   
        return m_size;
    }

    iterator maximum() const {	
        return (empty() ) ? end() : iterator(m_root->maximum() ) ;	
    }
    iterator min() const {	
        return (empty() ) ? end() : iterator(m_root->minimum() ) ;	
    }

    iterator find(value_type in_value ) const {
        node_type* pos = m_root ;
        while (pos != SENTINEL ) {
            if (pos->in_value == in_value )
                return iterator<T>(pos ) ;
            if (CMP()(in_value, pos->in_value ) )
                pos = pos->m_left ;
            else
                pos = pos->m_right ;
        }
        return end() ;
    }
    iterator insert(value_type in_value ) {
        return iterator(insert(new node_type(in_value, SENTINEL, RED ) ) ) ;
    }
    iterator insert(iterator& it, const_reference in_value ) {
        return insert(in_value ) ;
    }
    template< typename InputIter >
    void insert(InputIter in_beg, InputIter in_end ) {
        for (InputIter idx = in_beg ; idx != in_end ; ++idx )
            insert(idx->in_value ) ;
    }
    void remove(value_type in_value ) {
        iterator pos = find(in_value ) ;
        if (pos != end() )
            remove(pos ) ;
    }
private:
    node_type*	m_root ;
    size_type	m_size ;

    void left_rotate(node_type* in_node ) {
        if (in_node->m_right != SENTINEL ) {
            node_type* right_son = in_node->m_right ;
            if ((in_node->m_right = in_node->m_right->m_left ) != SENTINEL )
                in_node->m_right->m_parent = in_node ;
            if (in_node->m_parent == SENTINEL )
                m_root = right_son ;
            else if (in_node->is_left_son() )
                in_node->m_parent->m_left = right_son ;
            else
                in_node->m_parent->m_right = right_son ;	
            right_son->m_parent = in_node->m_parent ;
            in_node->m_parent = right_son ;
            right_son->m_left = in_node ;
        }
    }
    void right_rotate(node_type* in_node ) {
        if (in_node->m_left != SENTINEL ) {
            node_type* left_son = in_node->m_left ;
            if ((in_node->m_left = in_node->m_left->m_right ) != SENTINEL )
                in_node->m_left->m_parent = in_node ;
            if (in_node->m_parent == SENTINEL )
                m_root = left_son ;
            else if (in_node->is_left_son() )
                in_node->m_parent->m_left = left_son ;
            else
                in_node->m_parent->m_right = left_son ;	
            left_son->m_parent = in_node->m_parent ;
            in_node->m_parent = left_son ;
            left_son->m_right = in_node ;
        }
    }
    node_type* raw_insert(node_type* in_node ) {
        ++m_size ;
        if (empty() ) {
            in_node->m_parent = SENTINEL ;
            m_root = in_node ;
            return m_root ;
        }
        node_type* pos = m_root ;
        do {
            if (CMP()(in_node->m_value, pos->m_value ) ) {
                if (pos->m_left != SENTINEL )
                    pos = pos->m_left ;
                else{
                    pos->m_left = in_node ;
                    in_node->m_parent = pos ;
                    break ;
                }
            } else {
                if (pos->m_right != SENTINEL )
                    pos = pos->m_right ;
                else {
                    pos->m_right = in_node ;
                    in_node->m_parent = pos ;
                    break ;
                }
            }
        } while (true ) ;

        return pos ;
    }

    node_type* insert(node_type* in_node ) {    
        raw_insert(in_node ) ;
        in_node->m_color = RED ;
        while ((in_node->m_parent != m_root ) && 
            (in_node->m_parent->m_color == RED ) ) {
                if (in_node->m_parent->is_left_son() ) {
                    node_type* uncle = in_node->m_parent->m_parent->m_right ;
                    if (uncle->m_color == RED ) {
                        in_node->m_parent->m_color = BLACK ;
                        uncle->m_color = BLACK ;
                        in_node->m_parent->m_parent->m_color = RED ;
                        in_node = in_node->m_parent->m_parent ;
                    } else {
                        if (in_node->is_right_son() ) {
                            in_node = in_node->m_parent ;
                            left_rotate(in_node ) ;
                        }
                        in_node->m_parent->m_color = BLACK ;
                        in_node->m_parent->m_parent->m_color = RED ;
                        right_rotate(in_node->m_parent->m_parent ) ;
                    }
                } else if (in_node->m_parent->is_right_son() ) {
                    node_type* uncle = in_node->m_parent->m_parent->m_left ;
                    if (uncle->m_color == RED ) {
                        in_node->m_parent->m_color = BLACK ;
                        uncle->m_color = BLACK ;
                        in_node->m_parent->m_parent->m_color = RED ;
                        in_node = in_node->m_parent->m_parent ;
                    } else {
                        if (in_node->is_left_son() ) {
                            in_node = in_node->m_parent ;
                            right_rotate(in_node ) ;
                        }
                        in_node->m_parent->m_color = BLACK ;
                        in_node->m_parent->m_parent->m_color = RED ;
                        left_rotate(in_node->m_parent->m_parent ) ;
                    }
                }
        }
        m_root->m_color = BLACK ;
        _verifiy();
        return in_node ;
    }
    iterator remove(iterator& rhs ) {    
        node_type* sub ;
        iterator next(rhs ) ;
        ++next ;
        --m_size ;
        if ((rhs->m_left != SENTINEL ) && (rhs->m_right != SENTINEL ) ) {
            node_type* new_parent = rhs->successor() ;
            new_parent->m_left = rhs->m_left ;
            rhs->m_left->m_parent = new_parent ;
            sub = rhs->m_right ;
        } else
            sub = (rhs->m_left != SENTINEL ) ? rhs->m_left : rhs->m_right ;
        if (rhs->is_left_son() )
            rhs->m_parent->m_left = sub ;
        else if (rhs->is_right_son() )
            rhs->m_parent->m_right = sub ;
        else
            m_root = sub ;
        sub->m_parent = rhs->m_parent ;
        if (sub->m_color == BLACK ) {
            sub = (sub->m_left != SENTINEL ) ? sub->m_left : sub->m_right ;
            node_type* sibling ;
            while ((sub != m_root ) && (sub->m_color == BLACK ) ) {
                if (sub->is_left_son() ) {
                    sibling = sub->m_parent->m_right ;
                    if (sibling->m_color == RED ) {
                        sibling->m_color = BLACK ;
                        sub->m_parent->m_color = RED ;
                        left_rotate(sub->m_parent ) ;
                        sibling = sub->m_parent->m_right ;
                    }
                    if ((sibling->m_left->m_color == BLACK ) && 
                        (sibling->m_right->m_color == BLACK ) ) {
                            sibling->m_color = RED ;
                            sub = sub->m_parent ;
                    } else {
                        if (sibling->m_right->m_color == BLACK ) {
                            sub->m_left->m_color = BLACK ;
                            sibling->m_color = RED ;
                            right_rotate(sibling ) ;
                            sibling = sub->m_parent->m_right ;
                        }
                        sibling->m_color = sub->m_parent->m_color ;
                        sub->m_parent->m_color = BLACK ;
                        sibling->m_right->m_color = BLACK ;
                        left_rotate(sub->m_parent ) ;
                        sub = m_root ;
                    }
                } else {
                    sibling = sub->m_parent->m_left ;
                    if (sibling->m_color == RED ) {
                        sibling->m_color = BLACK ;
                        sub->m_parent->m_color = RED ;
                        right_rotate(sub->m_parent ) ;
                        sibling = sub->m_parent->m_left ;
                    }
                    if ((sibling->m_left->m_color == BLACK ) && 
                        (sibling->m_right->m_color == BLACK ) ) {
                            sibling->m_color = RED ;
                            sub = sub->m_parent ;
                    } else {
                        if (sibling->m_left->m_color == BLACK ) {
                            sub->m_right->m_color = BLACK ;
                            sibling->m_color = RED ;
                            left_rotate(sibling ) ;
                            sibling = sub->m_parent->m_left ;
                        }
                        sibling->m_color = sub->m_parent->m_color ;
                        sub->m_parent->m_color = BLACK ;
                        sibling->m_left->m_color = BLACK ;
                        right_rotate(sub->m_parent ) ;
                        sub = m_root ;
                    }
                }
            }
        }
        _verifiy();
        return next ;
    }
    bool _node_color(node_type* n) {
    	return n == SENTINEL ? BLACK : n->m_color;
	}

    void _verifiy() {
        assert ( _node_color(m_root) == BLACK );
        _verify_colors_for_each_node(m_root);
        _verifiy_colors_for_relationaship(m_root);
        _verify_path_count(m_root);
    }
    void _verify_colors_for_each_node(node_type *n) {
        assert( _node_color(n) == RED || 
                _node_color(n) == BLACK);
        if ( n == SENTINEL) return;
        _verify_colors_for_each_node(n->m_left);
        _verify_colors_for_each_node(n->m_right);
    }
    void _verifiy_colors_for_relationaship(node_type *n) {
        if ( _node_color(n) == RED ) {
            assert (_node_color(n->m_left)   == BLACK);
            assert (_node_color(n->m_right)  == BLACK);
            assert (_node_color(n->m_parent) == BLACK);
        }
        if ( n == SENTINEL) return;
        _verifiy_colors_for_relationaship(n->m_left);
        _verifiy_colors_for_relationaship(n->m_right);
    }
    void __verify_path_count(node_type *n,int black_count, 
                                          int* path_black_count) {
        if (_node_color(n) == BLACK) {
            black_count++;
        }
        if (n == SENTINEL) {
            if (*path_black_count == -1) {
                *path_black_count = black_count;
            } else {
                assert (black_count == *path_black_count);
            }
            return;
        }
        __verify_path_count(n->m_left, black_count, path_black_count);
        __verify_path_count(n->m_right, black_count, path_black_count);

    }
    void _verify_path_count(node_type *n) {
        int black_count_path = -1;
        __verify_path_count(m_root, 0, &black_count_path);
    }
};

template< typename T, typename CMP>
typename tree< T, CMP>::node_type* tree< T, CMP>::SENTINEL = 
                                          tree< T,CMP>::node_type::SENTINEL ;

#endif
