package grpl.pathfinder.util;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public abstract class NativeResourceTest {

    protected abstract NativeResource nativeResource();

    @Test
    void testClose() throws Exception {
        assertFalse(nativeResource().closed());
        nativeResource().close();
        assertTrue(nativeResource().closed());
    }

}
