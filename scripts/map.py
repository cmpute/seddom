import msgpack
import struct
import numpy as np
import sqlite3
import math
from collections import namedtuple, OrderedDict

Node = namedtuple("SeddomNode", ['timestamp', 'state', 'probs'])
SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE = 10

def morton(x):
    '''
    Generated 3D morton codes for a N*3 input
    Reference:
    https://blog.claude.nl/tech/timing-morton-code-on-python-on-apple-silicon/
    '''
    # for 32 bit:
    # x = (x | (x << 16)) & 0x030000FF
    # x = (x | (x <<  8)) & 0x0300F00F
    # x = (x | (x <<  4)) & 0x030C30C3
    # x = (x | (x <<  2)) & 0x09249249

    # for 64 bit:
    x = (x | x << 32) & 0x001f00000000ffff
    x = (x | x << 16) & 0x001f0000ff0000ff
    x = (x | x << 8)  & 0x100f00f00f00f00f
    x = (x | x << 4)  & 0x10c30c30c30c30c3
    x = (x | x << 2)  & 0x1249249249249249
    x = x << np.arange(3, dtype="u8")  
    return np.bitwise_or.reduce(x, axis=-1)

class SeddoMap:
    def __init__(self, database_path):
        self.engine = sqlite3.connect(database_path)
        self.blocks = OrderedDict()
        self.blocks_limit = 4096 # max number of loaded blocks
        
        # get configs from database
        configs = dict(self.engine.execute("SELECT * FROM meta"))
        self.block_depth = int(configs['block_depth'])
        self.chunk_depth = int(configs['chunk_depth'])
        self.resolution = float(configs['resolution'])
        self.semantic_class = configs['semantic_class']
        self.semantic_count = int(configs['semantic_count'])

        # caculate related params
        self.block_size = 2**(self.block_depth-1) * self.resolution
        self.chunk_size = 2**(self.block_depth + self.chunk_depth-2) * self.resolution
        self.map_size = self.block_size * 2**20
        self.map_origin = np.array([0, 0, 0], dtype=float)

    def __del__(self):
        self.engine.close()

    def loc_to_block(self, position):
        """
        Convert position (x,y,z) to block index (ix,iy,iz)
        """
        return (np.array(position, dtype='f4') / self.block_size + 524288.5).astype('u8')
        # return np.bitwise_or.reduce(pos_int << (np.arange(3, dtype="u8") * 20), axis=-1)

    def load_block(self, key_xyz):
        """
        Load a block into the cache. key_xyz should be a singe morton key.

        Currently all chunks are loaded from the x0y0z0 table
        """
        if key_xyz in self.blocks:
            return self.blocks[key_xyz]

        table = "x0y0z0"
        result = self.engine.execute("SELECT last_update, data FROM '%s' WHERE hashkey = %d;" % (table, key_xyz)).fetchall()
        if not result:
            return None

        last_update, data = result[0]
        def parse_node(data):
            if data is None:
                return None

            arrtype = data.code - SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE
            if arrtype == 1:
                (state, sem, psum, pfree, psem) = struct.unpack(">BBfff", data.data)
                semarr = np.array([pfree] + [psem if i == sem else (psum-pfree-psem)/(self.semantic_count-2)
                                for i in range(1, self.semantic_count)], dtype="f4")
                return Node(last_update, state, semarr)
            elif arrtype == 2:
                (state, sem, psum, psem) = struct.unpack(">BBff", data.data)
                semarr = np.array([psem if i == sem else (psum-psem)/(self.semantic_count-1)
                                for i in range(self.semantic_count)], dtype="f4")
                return Node(last_update, state, semarr)
            else:
                raise NotImplementedError("only node type 1,2 is supported right now, get %d" % arrtype)


        if len(self.blocks) == self.blocks_limit:
            self.blocks.popitem()
        self.blocks[key_xyz] = [parse_node(n) for n in msgpack.unpackb(data)]
        return self.blocks[key_xyz]

    def query_block(self, pos, key):
        """
        :param pos: N*3 position
        :param key: (ix, iy, iz)
        """
        morton_key = morton(key)
        if morton_key not in self.blocks:
            return None

        block_center = (np.array(key, dtype="f4") - 524288) * self.block_size
        cell_count = 2 ** (self.block_depth - 1)
        pos = ((pos - block_center[np.newaxis, :]) / self.resolution + cell_count / 2).astype(int)

        if np.any(pos < 0) or np.any(pos > cell_count):
            raise ValueError("The position is not in this block")

        index = morton(np.fliplr(pos.astype('u8'))) # _search_impl
        index += 8**(self.block_depth-1) // 7 # depth_index_to_index

        block = self.blocks[morton_key]
        return [np.argmax(block[i].probs) if block[i] else 0 for i in index]

    def query(self, position):
        """
        Query the states at the given positions (N*3)
        """
        position = np.asarray(position)
        if position.ndim == 1:
            position = np.array([position])

        block_pos = self.loc_to_block(position)
        block_id = morton(block_pos) # convert to morton id
        sem_values = np.empty_like(block_id)
        
        # find unique chunk_ids
        for key, kid in zip(*np.unique(block_id, return_index=True)):
            self.load_block(key)
            imask = block_id == key
            sem_values[imask] = self.query_block(position[imask], block_pos[kid]) or 0

        return sem_values
