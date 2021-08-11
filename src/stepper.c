st_prep_buffer () {
    while(segment_buffer_tail!=segment_buffer_head) {
        if(pl_block==NULL){
            pl_block = plan_get_current_block();
        }
        if (pl_block == NULL) { return; }
        st_prep_block = &st_block_buffer[prep.st_block_index];
        
    }
}